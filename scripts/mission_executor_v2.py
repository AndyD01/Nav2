#!/usr/bin/env python3
"""
mission_executor.py
Executa misiuni de docking pentru NOUZEN.
Suport approach points: robotul navigheaza la un punct favorabil
inainte de docking pentru aliniere optima.

Folosire:
  python3 mission_executor.py <mission_name>
  python3 mission_executor.py --list
  python3 mission_executor.py --topic
"""

import threading
import argparse
import json
import math
import os
import sys
import time
import yaml
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import DockRobot, UndockRobot, NavigateToPose
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import csv
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# ============================================================
# CONFIGURATIE
# ============================================================
DEFAULT_LOG_DIR = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/logs'
)
MISSION_FILE = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/config/mission.yaml'
)
DOCK_DATABASE_FILE = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/config/dock_database.yaml'
)

DOCK_TAG_MAP = {
    'nouzen_dock_station': 0,
    'home':                1,
    'input_a':             2,
    'input_b':             3,
    'output_a':            4,
    'output_b':            5,
    'SCSS1':               6,
    'SCSS2':               7,
}

ACTION_TIMEOUT_GOAL   = 10.0
ACTION_TIMEOUT_DOCK   = 300.0
ACTION_TIMEOUT_UNDOCK = 30.0
ACTION_TIMEOUT_NAV    = 120.0
AMCL_TIMEOUT          = 10.0
TF_LOOKUP_TIMEOUT     = 5.0
# ============================================================


class MissionLogger:
    def __init__(self, ros_logger, log_dir=None, mission_name='unknown'):
        self.ros_logger = ros_logger
        self.log_file = None
        self.start_time = time.time()
        self.step_start = None

        os.makedirs(log_dir or DEFAULT_LOG_DIR, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join(
            log_dir or DEFAULT_LOG_DIR,
            f'mission_{mission_name}_{timestamp}.log'
        )
        self.log_file = open(log_path, 'w')
        self._file(f'Mission: {mission_name}')
        self._file(f'Date: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
        self._file(f'Log: {log_path}')
        self._file('=' * 60)
        self.ros_logger.info(f'Log: {log_path}')

        # --- CSV structured log ---
        csv_path = os.path.join(
            log_dir or DEFAULT_LOG_DIR,
            f'mission_{mission_name}_{timestamp}.csv'
        )
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'mission_name', 'step_num', 'dock_id',
            'action',           # dock / undock / ap_nav
            'result',           # SUCCESS / FAIL
            'error_code',       # 903-906, 999, timeout, ''
            'duration_sec',
            'pos_error_m',      # TF: dist base_link -> tag
            'ang_error_deg',    # TF: yaw error vs approach_yaw
            'robot_x', 'robot_y', 'robot_yaw_deg',
            'notes',
        ])
        self.csv_file.flush()
        self.csv_mission_name = mission_name
        self.ros_logger.info(f'CSV: {csv_path}')

    def _elapsed(self):
        return time.time() - self.start_time

    def _file(self, msg, level='INFO'):
        if self.log_file:
            ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            elapsed = self._elapsed()
            self.log_file.write(
                f'[{ts}] [{elapsed:8.2f}s] [{level:<7}] {msg}\n'
            )
            self.log_file.flush()

    def _terminal(self, level, msg):
        if level == 'INFO':
            self.ros_logger.info(msg)
        elif level == 'WARN':
            self.ros_logger.warn(msg)
        elif level == 'ERROR':
            self.ros_logger.error(msg)

    def info(self, msg, terminal=True):
        self._file(msg, 'INFO')
        if terminal:
            self._terminal('INFO', msg)

    def warn(self, msg, terminal=True):
        self._file(msg, 'WARN')
        if terminal:
            self._terminal('WARN', msg)

    def error(self, msg, terminal=True):
        self._file(msg, 'ERROR')
        if terminal:
            self._terminal('ERROR', msg)

    def detail(self, msg):
        self._file(msg, 'DETAIL')

    def section(self, title, terminal=True):
        sep = '=' * 60
        self._file(sep)
        self._file(f'  {title}', 'SECTION')
        self._file(sep)
        if terminal:
            self.ros_logger.info(f'--- {title} ---')

    def step_start_timer(self):
        self.step_start = time.time()

    def step_elapsed(self):
        if self.step_start:
            return time.time() - self.step_start
        return 0.0

    def pose(self, label, x, y, yaw_rad=None):
        if yaw_rad is not None:
            self.detail(
                f'{label}: x={x:.4f}, y={y:.4f}, '
                f'yaw={yaw_rad:.4f}rad ({math.degrees(yaw_rad):.1f}deg)'
            )
        else:
            self.detail(f'{label}: x={x:.4f}, y={y:.4f}')

    def csv_row(self, step_num, dock_id, action, result,
                error_code='', duration_sec=0.0,
                pos_error_m='', ang_error_deg='',
                robot_x='', robot_y='', robot_yaw_deg='',
                notes=''):
        """Write one structured row to the CSV experiment log."""
        if not hasattr(self, 'csv_writer') or self.csv_writer is None:
            return
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.csv_writer.writerow([
            ts, self.csv_mission_name, step_num, dock_id,
            action, result, error_code,
            f'{duration_sec:.2f}' if isinstance(duration_sec, float) else duration_sec,
            f'{pos_error_m:.4f}' if isinstance(pos_error_m, float) else pos_error_m,
            f'{ang_error_deg:.1f}' if isinstance(ang_error_deg, float) else ang_error_deg,
            f'{robot_x:.4f}' if isinstance(robot_x, float) else robot_x,
            f'{robot_y:.4f}' if isinstance(robot_y, float) else robot_y,
            f'{robot_yaw_deg:.1f}' if isinstance(robot_yaw_deg, float) else robot_yaw_deg,
            notes,
        ])
        self.csv_file.flush()

    def close(self):
        total = self._elapsed()
        self._file('=' * 60)
        self._file(f'Total duration: {total:.1f}s')
        self._file('Log closed')
        if self.log_file:
            self.log_file.close()
            self.log_file = None
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.csv_file = None


def quaternion_to_yaw(qz, qw):
    return 2.0 * math.atan2(qz, qw)


def load_mission_file():
    with open(MISSION_FILE, 'r') as f:
        return yaml.safe_load(f)


def load_dock_database():
    with open(DOCK_DATABASE_FILE, 'r') as f:
        return yaml.safe_load(f)


class MissionExecutor(Node):
    def __init__(self, log, dock_db, mode='cli'):
        super().__init__('mission_executor')
        self.log = log
        self.dock_db = dock_db
        self.mode = mode

        # Action clients
        self.dock_client   = ActionClient(self, DockRobot,   '/dock_robot')
        self.undock_client = ActionClient(self, UndockRobot, '/undock_robot')
        self.nav_client    = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.param_client  = self.create_client(
            SetParameters, '/dock_pose_publisher/set_parameters'
        )

        # TF buffer for dock error measurement
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Topic mode
        if self.mode == 'topic':
            self.goal_sub = self.create_subscription(
                String, '/mission_executor/goal', self.on_goal, 10
            )
            self.status_pub = self.create_publisher(
                String, '/mission_executor/status', 10
            )
            self.result_pub = self.create_publisher(
                String, '/mission_executor/result', 10
            )
            self.busy = False
            self.get_logger().info('mission_executor in mod TOPIC.')
            self._cb_executor = rclpy.executors.MultiThreadedExecutor()
            self._cb_executor.add_node(self)
            self._spin_lock = threading.Lock()

    # ==============================================================
    # APPROACH POINT
    # ==============================================================

    def navigate_to_approach_point(self, dock_id):
        """Navigheaza robotul la approach point inainte de dock."""
        dock_entry = self.dock_db.get('docks', {}).get(dock_id, {})
        ap = dock_entry.get('approach_point')
        if not ap:
            self.log.detail(f'AP: {dock_id} nu are approach_point, skip.')
            return True

        x, y = ap[0], ap[1]
        yaw = ap[2] if len(ap) > 2 else 0.0

        self.log.info(
            f'AP [{dock_id}]: navigare la ({x:.2f}, {y:.2f}, '
            f'yaw={math.degrees(yaw):.1f}deg)',
            terminal=True
        )

        # Pozitia curenta inainte de AP
        pose = self.get_robot_pose()
        if pose:
            dist_to_ap = math.sqrt((pose[0] - x)**2 + (pose[1] - y)**2)
            self.log.detail(
                f'AP [{dock_id}]: pozitie curenta ({pose[0]:.2f}, {pose[1]:.2f}), '
                f'distanta pana la AP: {dist_to_ap:.2f}m'
            )
            if dist_to_ap < 0.3:
                self.log.info(
                    f'AP [{dock_id}]: deja la AP (dist={dist_to_ap:.2f}m), skip navigatie.',
                    terminal=True
                )
                return True

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.log.error(f'AP [{dock_id}]: NavigateToPose server indisponibil.')
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.log.detail(f'AP [{dock_id}]: trimit NavigateToPose goal...')
        ap_start = time.time()

        future = self.nav_client.send_goal_async(goal)
        self._spin_future(future, timeout_sec=ACTION_TIMEOUT_GOAL)

        handle = future.result()
        if not handle:
            self.log.warn(f'AP [{dock_id}]: goal handle None, skip.')
            return True
        if not handle.accepted:
            self.log.warn(f'AP [{dock_id}]: goal respins de server, skip.')
            return True

        self.log.detail(f'AP [{dock_id}]: goal acceptat, navighez...')

        result_future = handle.get_result_async()
        self._spin_future(result_future, timeout_sec=ACTION_TIMEOUT_NAV)

        ap_duration = time.time() - ap_start

        if result_future.done():
            result = result_future.result()
            if result and result.status == 4:  # SUCCEEDED
                pose_after = self.get_robot_pose()
                if pose_after:
                    actual_dist = math.sqrt(
                        (pose_after[0] - x)**2 + (pose_after[1] - y)**2
                    )
                    self.log.info(
                        f'AP [{dock_id}]: ATINS in {ap_duration:.1f}s, '
                        f'pozitie ({pose_after[0]:.2f}, {pose_after[1]:.2f}), '
                        f'eroare: {actual_dist:.2f}m',
                        terminal=True
                    )
                else:
                    self.log.info(
                        f'AP [{dock_id}]: ATINS in {ap_duration:.1f}s',
                        terminal=True
                    )
                return True
            else:
                status = result.status if result else 'None'
                self.log.warn(
                    f'AP [{dock_id}]: ESUAT (status={status}) dupa {ap_duration:.1f}s, '
                    f'continui cu dock direct.',
                    terminal=True
                )
                return True
        else:
            self.log.warn(
                f'AP [{dock_id}]: TIMEOUT dupa {ap_duration:.1f}s, '
                f'continui cu dock direct.',
                terminal=True
            )
            return True

    # ==============================================================
    # CORE
    # ==============================================================

    def get_robot_pose(self):
        """Citeste pozitia robotului din /amcl_pose."""
        pose_data = {'done': False, 'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        def cb(msg):
            p = msg.pose.pose
            pose_data['x']   = p.position.x
            pose_data['y']   = p.position.y
            pose_data['yaw'] = quaternion_to_yaw(
                p.orientation.z, p.orientation.w
            )
            pose_data['done'] = True

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        sub   = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', cb, qos
        )
        start = time.time()
        while not pose_data['done'] and (time.time() - start) < AMCL_TIMEOUT:
            if self.mode == 'topic':
                # In topic mode spin-ul ruleaza deja pe main thread,
                # callback-ul va fi procesat automat
                time.sleep(0.05)
            else:
                rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(sub)

        if pose_data['done']:
            self.log.pose(
                'Robot pose',
                pose_data['x'], pose_data['y'], pose_data['yaw']
            )
            return pose_data['x'], pose_data['y'], pose_data['yaw']

        self.log.warn('Nu am putut citi pozitia AMCL.', terminal=False)
        return None

    def get_dock_error(self, dock_id):
        """
        TF lookup: base_link -> tag frame dupa dock reusit.
        Returneaza (pos_error_m, ang_error_deg) sau (None, None).
        pos_error_m  = distanta euclidiana XY intre base_link si tag
        ang_error_deg = diferenta yaw fata de approach_yaw din dock_db
        """
        tag_id = DOCK_TAG_MAP.get(dock_id)
        if tag_id is None:
            return None, None

        target_frame = 'base_link'
        source_frame = f'tag36h11:{tag_id}'

        # Spin un pic ca TF-ul sa fie fresh
        if self.mode == 'topic':
            time.sleep(0.5)
        else:
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.05)

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                target_frame, source_frame, now,
                timeout=Duration(seconds=TF_LOOKUP_TIMEOUT)
            )
            tx = trans.transform.translation.x
            ty = trans.transform.translation.y
            pos_error = math.sqrt(tx**2 + ty**2)

            # Yaw din quaternion
            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            tf_yaw = quaternion_to_yaw(qz, qw)

            # Compara cu approach_yaw din dock_db
            dock_entry = self.dock_db.get('docks', {}).get(dock_id, {})
            approach_yaw = dock_entry.get('approach_yaw', 0.0)
            ang_error = math.degrees(tf_yaw - approach_yaw)
            # Normalizeaza la [-180, 180]
            ang_error = (ang_error + 180) % 360 - 180

            self.log.info(
                f'DOCK ERROR [{dock_id}]: pos={pos_error:.4f}m, '
                f'ang={ang_error:.1f}deg '
                f'(tf: x={tx:.4f}, y={ty:.4f}, yaw={math.degrees(tf_yaw):.1f}deg)',
                terminal=True
            )
            return pos_error, ang_error

        except Exception as e:
            self.log.warn(
                f'TF lookup esuat [{target_frame}->{source_frame}]: {e}',
                terminal=True
            )
            return None, None

    def set_tag(self, dock_id):
        """Seteaza dock_tag_id pe publisher."""
        if dock_id not in DOCK_TAG_MAP:
            self.log.error(f'dock_id "{dock_id}" necunoscut in DOCK_TAG_MAP.')
            return False

        if not self.param_client.wait_for_service(timeout_sec=5.0):
            self.log.error('dock_pose_publisher indisponibil.')
            return False

        tag_id = DOCK_TAG_MAP[dock_id]
        dock_entry = self.dock_db.get('docks', {}).get(dock_id, {})
        approach_yaw = dock_entry.get('approach_yaw', 0.0)

        self.log.detail(
            f'Set publisher: tag36h11:{tag_id}, '
            f'approach_yaw={math.degrees(approach_yaw):.1f}deg'
        )

        req = SetParameters.Request()
        req.parameters = [
            Parameter(
                name='dock_tag_id',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_INTEGER,
                    integer_value=tag_id
                )
            ),
            Parameter(
                name='approach_yaw',
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=approach_yaw
                )
            ),
        ]

        future = self.param_client.call_async(req)
        self._spin_future(future, timeout_sec=3.0)
        return True

    def do_dock(self, dock_id):
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self.log.error('Docking server indisponibil.')
            return False

        goal = DockRobot.Goal()
        goal.dock_id = dock_id
        goal.use_dock_id = True
        goal.navigate_to_staging_pose = True

        self.log.detail(f'Trimit dock goal: dock_id={dock_id}')

        future = self.dock_client.send_goal_async(goal)
        self._spin_future(future, timeout_sec=ACTION_TIMEOUT_GOAL)

        handle = future.result()
        if not handle or not handle.accepted:
            self.log.error(f'Dock goal respins pentru {dock_id}.')
            return False

        self.log.detail('Dock goal acceptat, navighez la staging...')

        result_future = handle.get_result_async()
        self._spin_future(result_future, timeout_sec=ACTION_TIMEOUT_DOCK)

        result = result_future.result()
        if result and result.result.success:
            self.log.detail(
                f'Dock reusit la {dock_id} '
                f'(retries={result.result.num_retries}, '
                f'durata={self.log.step_elapsed():.1f}s)'
            )
            self._last_dock_error_code = ''
            self._last_dock_retries = result.result.num_retries
            return True

        error_map = {
            901: 'DOCK_NOT_IN_DB',
            902: 'DOCK_NOT_VALID',
            903: 'FAILED_TO_STAGE',
            904: 'FAILED_TO_DETECT_DOCK',
            905: 'FAILED_TO_CONTROL',
            906: 'FAILED_TO_CHARGE',
            999: 'UNKNOWN',
        }
        error_code = result.result.error_code if result else 'timeout'
        error_str  = error_map.get(error_code, str(error_code))
        self.log.detail(
            f'Dock esuat la {dock_id}: {error_str} ({error_code}), '
            f'durata={self.log.step_elapsed():.1f}s'
        )
        self._last_dock_error_code = f'{error_str}({error_code})'
        self._last_dock_retries = 0
        return False

    def do_undock(self, dock_id):
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.log.error('Undocking server indisponibil.')
            return False

        dock_type = self.dock_db.get('docks', {}).get(
            dock_id, {}
        ).get('type', 'charging_dock')

        goal = UndockRobot.Goal()
        goal.dock_type = dock_type

        self.log.detail(f'Trimit undock goal: dock_type={dock_type}')

        future = self.undock_client.send_goal_async(goal)
        self._spin_future(future, timeout_sec=ACTION_TIMEOUT_GOAL)

        handle = future.result()
        if not handle or not handle.accepted:
            self.log.error('Undock goal respins.')
            return False

        result_future = handle.get_result_async()
        self._spin_future(result_future, timeout_sec=ACTION_TIMEOUT_UNDOCK)

        result = result_future.result()
        if result and result.result.success:
            self.log.detail(
                f'Undock reusit (durata={self.log.step_elapsed():.1f}s)'
            )
            return True

        self.log.detail('Undock esuat.')
        return False

    # ==============================================================
    # CLI MODE
    # ==============================================================

    def execute_mission(self, mission_name, mission):
        total = len(mission)
        mission_start = time.time()
        results = []

        self.log.section(f'MISIUNE: {mission_name} ({total} pasi)')

        pose = self.get_robot_pose()
        if pose:
            self.log.detail(
                f'Pozitie initiala: x={pose[0]:.3f}, y={pose[1]:.3f}, '
                f'yaw={math.degrees(pose[2]):.1f}deg'
            )

        for i, step in enumerate(mission):
            dock_id    = step['dock_id']
            dwell_time = step.get('dwell_time', 0)
            step_num   = f'{i+1}/{total}'

            self.log.section(f'PAS {step_num}: {dock_id}', terminal=True)
            self.log.step_start_timer()

            # 1. Set tag
            self.log.detail(f'[{step_num}] Setez tag pentru {dock_id}...')
            if not self.set_tag(dock_id):
                self.log.error(
                    f'[{step_num}] Set tag esuat.',
                    terminal=True
                )
                results.append((dock_id, 'SET_TAG_FAILED'))
                break

            # 2. Approach point
            self.log.detail(f'[{step_num}] Verific approach point pentru {dock_id}...')
            ap_ok = self.navigate_to_approach_point(dock_id)
            if not ap_ok:
                self.log.warn(
                    f'[{step_num}] AP esuat, incerc dock direct.',
                    terminal=True
                )

            # 3. Dock
            self.log.info(
                f'[{step_num}] Dock -> {dock_id}...', terminal=True
            )
            dock_ok = self.do_dock(dock_id)

            dock_duration = self.log.step_elapsed()

            if dock_ok:
                self.log.info(
                    f'[{step_num}] Dock reusit '
                    f'({dock_duration:.1f}s)',
                    terminal=True
                )
                results.append((dock_id, 'SUCCESS'))

                pose = self.get_robot_pose()
                if pose:
                    self.log.detail(
                        f'Pozitie dupa dock: x={pose[0]:.3f}, '
                        f'y={pose[1]:.3f}'
                    )

                # --- TF dock error measurement ---
                pos_err, ang_err = self.get_dock_error(dock_id)

                self.log.csv_row(
                    step_num=step_num, dock_id=dock_id,
                    action='dock', result='SUCCESS',
                    error_code='',
                    duration_sec=dock_duration,
                    pos_error_m=pos_err if pos_err is not None else '',
                    ang_error_deg=ang_err if ang_err is not None else '',
                    robot_x=pose[0] if pose else '',
                    robot_y=pose[1] if pose else '',
                    robot_yaw_deg=math.degrees(pose[2]) if pose else '',
                    notes=f'retries={self._last_dock_retries}',
                )
            else:
                self.log.error(
                    f'[{step_num}] Dock esuat la {dock_id} '
                    f'-- opresc misiunea.',
                    terminal=True
                )
                results.append((dock_id, 'DOCK_FAILED'))

                self.log.csv_row(
                    step_num=step_num, dock_id=dock_id,
                    action='dock', result='FAIL',
                    error_code=getattr(self, '_last_dock_error_code', ''),
                    duration_sec=dock_duration,
                )
                break

            # 4. Dwell
            if dwell_time > 0:
                self.log.info(
                    f'[{step_num}] Stationare {dwell_time}s...',
                    terminal=True
                )
                time.sleep(dwell_time)

            # 5. Undock
            if i < total - 1:
                next_dock = mission[i + 1]['dock_id']
                self.log.info(
                    f'[{step_num}] Undock, urmator: {next_dock}',
                    terminal=True
                )
                self.log.step_start_timer()
                undock_ok = self.do_undock(dock_id)

                undock_duration = self.log.step_elapsed()

                if not undock_ok:
                    self.log.error(
                        f'[{step_num}] Undock esuat.',
                        terminal=True
                    )
                    results.append((dock_id, 'UNDOCK_FAILED'))
                    self.log.csv_row(
                        step_num=step_num, dock_id=dock_id,
                        action='undock', result='FAIL',
                        duration_sec=undock_duration,
                    )
                    break

                self.log.info(
                    f'[{step_num}] Undock reusit '
                    f'({undock_duration:.1f}s)',
                    terminal=True
                )

                # Log pozitie dupa undock
                pose = self.get_robot_pose()
                if pose:
                    self.log.detail(
                        f'Pozitie dupa undock: x={pose[0]:.3f}, '
                        f'y={pose[1]:.3f}, yaw={math.degrees(pose[2]):.1f}deg'
                    )

                self.log.csv_row(
                    step_num=step_num, dock_id=dock_id,
                    action='undock', result='SUCCESS',
                    duration_sec=undock_duration,
                    robot_x=pose[0] if pose else '',
                    robot_y=pose[1] if pose else '',
                    robot_yaw_deg=math.degrees(pose[2]) if pose else '',
                )

        # Summary
        total_duration = time.time() - mission_start
        self.log.section('SUMMARY', terminal=True)
        self.log.info(
            f'Durata totala: {total_duration:.1f}s', terminal=True
        )

        all_ok = all(r == 'SUCCESS' for _, r in results)
        for dock_id, result in results:
            status_str = 'OK' if result == 'SUCCESS' else f'FAIL ({result})'
            self.log.info(f'  {dock_id}: {status_str}', terminal=True)
            self.log.detail(f'Result: {dock_id} -> {result}')

        self.log.info(
            f'Misiune {"COMPLETA" if all_ok else "ESUATA"}',
            terminal=True
        )
        return all_ok

    # ==============================================================
    # TOPIC MODE
    # ==============================================================

    def publish_status(self, mission_id, state, dock_id='', step='',
                       elapsed=0.0, extra=None):
        if self.mode != 'topic':
            return
        status = {
            'mission_id': mission_id,
            'state': state,
            'dock_id': dock_id,
            'step': step,
            'elapsed_sec': round(elapsed, 1),
        }
        if extra:
            status.update(extra)
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_result(self, result_dict):
        if self.mode != 'topic':
            return
        msg = String()
        msg.data = json.dumps(result_dict)
        self.result_pub.publish(msg)

    def on_goal(self, msg):
        """Callback la /mission_executor/goal. Ruleaza misiunea pe thread separat."""
        if self.busy:
            self.get_logger().warn('Executor ocupat, goal ignorat.')
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error('JSON invalid pe /mission_executor/goal')
            return

        mission_id = data.get('mission_id', 'unknown')
        dock_ids   = data.get('dock_ids', [])
        dwell_times = data.get('dwell_times', [])

        if not dock_ids:
            self.publish_result({
                'mission_id': mission_id,
                'success': False,
                'error_msg': 'empty_dock_ids',
                'duration_sec': 0.0,
                'completed_docks': [],
                'metrics': {},
            })
            return

        mission = []
        for i, did in enumerate(dock_ids):
            dwell = dwell_times[i] if i < len(dwell_times) else 0
            mission.append({'dock_id': did, 'dwell_time': dwell})

        self.get_logger().info(
            f'Goal primit: {mission_id} {" -> ".join(dock_ids)}'
        )

        # Ruleaza misiunea pe thread separat ca sa nu blocheze spin()
        t = threading.Thread(
            target=self._run_mission_thread,
            args=(mission_id, mission),
            daemon=True
        )
        t.start()

    def execute_mission_topic(self, mission_id, mission):
        total = len(mission)
        mission_start = time.time()
        self._last_completed = []
        metrics = {}

        self.log.section(f'MISIUNE TOPIC: {mission_id} ({total} pasi)')
        self.publish_status(mission_id, 'starting', step=f'0/{total}')

        for i, step in enumerate(mission):
            dock_id    = step['dock_id']
            dwell_time = step.get('dwell_time', 0)
            step_num   = f'{i+1}/{total}'

            self.log.section(f'PAS {step_num}: {dock_id}')
            self.log.step_start_timer()

            # 1. Set tag
            self.publish_status(
                mission_id, 'setting_tag', dock_id=dock_id, step=step_num
            )
            if not self.set_tag(dock_id):
                metrics['error_msg'] = f'set_tag_failed:{dock_id}'
                metrics['total_sec'] = time.time() - mission_start
                return False, metrics

            # 2. Approach point
            self.publish_status(
                mission_id, 'approach_point', dock_id=dock_id, step=step_num
            )
            self.navigate_to_approach_point(dock_id)

            # 3. Dock
            self.publish_status(
                mission_id, 'docking', dock_id=dock_id, step=step_num
            )
            self.log.info(f'[{step_num}] Dock -> {dock_id}...')

            dock_ok = self.do_dock(dock_id)

            dock_duration = self.log.step_elapsed()

            if dock_ok:
                self.log.info(
                    f'[{step_num}] Dock reusit ({dock_duration:.1f}s)'
                )
                self._last_completed.append(dock_id)

                # --- TF dock error measurement ---
                pos_err, ang_err = self.get_dock_error(dock_id)
                pose = self.get_robot_pose()

                self.log.csv_row(
                    step_num=step_num, dock_id=dock_id,
                    action='dock', result='SUCCESS',
                    duration_sec=dock_duration,
                    pos_error_m=pos_err if pos_err is not None else '',
                    ang_error_deg=ang_err if ang_err is not None else '',
                    robot_x=pose[0] if pose else '',
                    robot_y=pose[1] if pose else '',
                    robot_yaw_deg=math.degrees(pose[2]) if pose else '',
                    notes=f'retries={self._last_dock_retries}',
                )
            else:
                self.log.error(f'[{step_num}] Dock esuat la {dock_id}.')
                metrics['error_msg'] = f'dock_failed:{dock_id}'
                metrics['total_sec'] = time.time() - mission_start

                self.log.csv_row(
                    step_num=step_num, dock_id=dock_id,
                    action='dock', result='FAIL',
                    error_code=getattr(self, '_last_dock_error_code', ''),
                    duration_sec=dock_duration,
                )
                return False, metrics

            # 4. Dwell
            if dwell_time > 0:
                self.publish_status(
                    mission_id, 'dwelling', dock_id=dock_id, step=step_num
                )
                self.log.info(f'[{step_num}] Stationare {dwell_time}s...')
                time.sleep(dwell_time)

            # 5. Undock
            if i < total - 1:
                next_dock = mission[i + 1]['dock_id']
                self.publish_status(
                    mission_id, 'undocking', dock_id=dock_id, step=step_num
                )
                self.log.info(f'[{step_num}] Undock, urmator: {next_dock}')
                self.log.step_start_timer()
                undock_ok = self.do_undock(dock_id)

                undock_duration = self.log.step_elapsed()

                if not undock_ok:
                    metrics['error_msg'] = f'undock_failed:{dock_id}'
                    metrics['total_sec'] = time.time() - mission_start
                    self.log.csv_row(
                        step_num=step_num, dock_id=dock_id,
                        action='undock', result='FAIL',
                        duration_sec=undock_duration,
                    )
                    return False, metrics

                self.log.info(
                    f'[{step_num}] Undock reusit ({undock_duration:.1f}s)'
                )
                self.log.csv_row(
                    step_num=step_num, dock_id=dock_id,
                    action='undock', result='SUCCESS',
                    duration_sec=undock_duration,
                )

        total_duration = time.time() - mission_start
        metrics['total_sec'] = round(total_duration, 1)

        self.log.section('SUMMARY')
        self.log.info(f'Misiune COMPLETA in {total_duration:.1f}s')
        self.publish_status(mission_id, 'complete', elapsed=total_duration)
        return True, metrics

    def _run_mission_thread(self, mission_id, mission):
        """Ruleaza misiunea pe un thread non-spin si publica result."""
        self.busy = True
        self.log = MissionLogger(
            self.get_logger(), mission_name=mission_id
        )

        success, metrics = self.execute_mission_topic(mission_id, mission)

        completed = self._last_completed if hasattr(self, '_last_completed') else []

        self.publish_result({
            'mission_id': mission_id,
            'success': success,
            'error_msg': '' if success else metrics.get('error_msg', 'dock_failed'),
            'duration_sec': round(metrics.get('total_sec', 0.0), 1),
            'completed_docks': completed,
            'metrics': metrics,
        })

        self.log.close()
        self.busy = False

    def _spin_future(self, future, timeout_sec=10.0):
        """
        Asteapta un future indiferent de context (main thread sau thread separat).
        In modul topic ruleaza din thread, deci nu poate folosi rclpy.spin_until_future_complete
        (ar crapa cu 'Executor is already spinning').
        Solutie: busy-wait cu spin_once pe un executor secundar.
        """
        if self.mode != 'topic':
            # In CLI mode, spin normal
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            return

        # In topic mode: busy-wait pe executor secundar
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)


def main():
    # Filtreaza argumentele ROS2 injectate de launch
    argv = sys.argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]

    parser = argparse.ArgumentParser(description='NOUZEN Mission Executor')
    parser.add_argument(
        'mission', nargs='?',
        help='Numele misiunii din mission.yaml'
    )
    parser.add_argument(
        '--list', action='store_true',
        help='Listeaza misiunile disponibile'
    )
    parser.add_argument(
        '--log-dir', type=str, default=DEFAULT_LOG_DIR,
        help=f'Director pentru log-uri (default: {DEFAULT_LOG_DIR})'
    )
    parser.add_argument(
        '--topic', action='store_true',
        help='Ruleaza in mod topic (asculta /mission_executor/goal)'
    )
    args = parser.parse_args(argv)  # <-- singura schimbare fata de ce aveai

    dock_db = load_dock_database()

    # --list
    if args.list:
        data = load_mission_file()
        missions = data.get('missions', {})
        print('Misiuni disponibile:')
        for name, steps in missions.items():
            docks = [s['dock_id'] for s in steps]
            print(f'  {name}: {" -> ".join(docks)}')
        return

    # --topic mode
    if args.topic:
        rclpy.init()
        tmp_node = rclpy.create_node('_tmp_logger_topic')
        log = MissionLogger(
            tmp_node.get_logger(), mission_name='topic_standby'
        )
        tmp_node.destroy_node()

        node = MissionExecutor(log, dock_db, mode='topic')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('mission_executor oprit.')
        finally:
            log.close()
            node.destroy_node()
            rclpy.shutdown()
        return

    # CLI mode
    if not args.mission:
        parser.print_help()
        sys.exit(1)

    data = load_mission_file()
    missions = data.get('missions', {})

    if args.mission not in missions:
        print(f'Misiunea "{args.mission}" nu exista.')
        print(f'Disponibile: {list(missions.keys())}')
        sys.exit(1)

    mission = missions[args.mission]

    rclpy.init()

    tmp_node = rclpy.create_node('_tmp_logger')
    log = MissionLogger(
        tmp_node.get_logger(),
        log_dir=args.log_dir,
        mission_name=args.mission
    )
    tmp_node.destroy_node()

    executor_node = MissionExecutor(log, dock_db)

    try:
        success = executor_node.execute_mission(args.mission, mission)
    except KeyboardInterrupt:
        log.warn('Misiune intrerupta de utilizator.', terminal=True)
        success = False
    finally:
        log.close()
        executor_node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()