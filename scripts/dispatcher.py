#!/usr/bin/env python3
"""
dispatcher.py
Nod ROS2 central pentru NOUZEN intralogistics.

Responsabilitati:
  - Asculta /station_events pentru station_ready
  - Mentine transport_queue cu rutare inteligenta
  - Trimite misiuni pe /mission_executor/goal
  - Asculta /mission_executor/result si /mission_executor/status
  - Notifica station_manager via /station_manager/cmd
  - Asculta /dispatcher/inject pentru comenzi operator
  - Publica /dispatcher/status (starea completa)
  - Retry logic, mission timeout, metrici detaliate
  - AMCL check la pornire si pre-dispatch
  - Skip mission, reason pe start/stop production

Pornire:
  python3 dispatcher.py
"""

import csv
import json
import math
import os
import signal
import sys
import time
import uuid
from collections import defaultdict
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import yaml

# ============================================================
# CONFIGURATIE
# ============================================================
CONFIG_FILE = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/config/station_config.yaml'
)
DOCK_DATABASE_FILE = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/config/dock_database.yaml'
)
LOG_DIR = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/logs'
)

# Stari productie
PROD_STOPPED = 'STOPPED'
PROD_RUNNING = 'RUNNING'
PROD_PAUSED  = 'PAUSED'

# Stari transport
TRANS_STOPPED = 'STOPPED'
TRANS_ACTIVE  = 'ACTIVE'
TRANS_PAUSED  = 'PAUSED'

# Stari robot
ROBOT_DOCKED_HOME    = 'DOCKED_HOME'
ROBOT_IDLE           = 'IDLE'
ROBOT_ACTIVE         = 'ACTIVE'
ROBOT_PAUSED         = 'PAUSED'
ROBOT_RETURNING_HOME = 'RETURNING_HOME'
ROBOT_RETRY          = 'RETRY'
ROBOT_EMERGENCY_STOP = 'EMERGENCY_STOP'

# Stari sistem
SYS_INITIALIZING     = 'INITIALIZING'
SYS_PRODUCTION_ONLY  = 'PRODUCTION_ONLY'
SYS_FULL_ACTIVE      = 'FULL_ACTIVE'
SYS_PAUSED           = 'PAUSED'
SYS_SHUTTING_DOWN    = 'SHUTTING_DOWN'
SYS_EMERGENCY_STOP   = 'EMERGENCY_STOP'

STATUS_PUBLISH_RATE  = 2.0   # Hz
QUEUE_CHECK_RATE     = 1.0   # Hz
MISSION_TIMEOUT_SEC  = 600.0
AMCL_HOME_THRESHOLD  = 0.5   # m - daca e mai departe, nu e docat
AMCL_DRIFT_THRESHOLD = 1.0   # m - drift maxim acceptat inainte de dispatch
# ============================================================


class DispatcherLogger:
    """Dual .log + .csv logger pentru dispatcher."""

    def __init__(self, ros_logger, log_dir=None):
        self.ros_logger = ros_logger
        self.start_time = time.time()

        log_dir = log_dir or LOG_DIR
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # .log file
        log_path = os.path.join(log_dir, f'dispatcher_{ts}.log')
        self.log_file = open(log_path, 'w')
        self._file('dispatcher pornit')
        self._file(f'Log: {log_path}')
        self._file('=' * 60)
        self.ros_logger.info(f'Log: {log_path}')

        # .csv file
        csv_path = os.path.join(log_dir, f'dispatcher_{ts}.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'elapsed_sec',
            'event',
            'mission_id',
            'input_id',
            'output_id',
            'material_type',
            'prod_state',
            'trans_state',
            'robot_state',
            'sys_state',
            'queue_size',
            'weight_kg',
            'duration_sec',
            'queue_wait_sec',
            'robot_x',
            'robot_y',
            'route_distance',
            'retry_count',
            'error_msg',
            'details',
        ])
        self.csv_file.flush()
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

    def info(self, msg, terminal=True):
        self._file(msg, 'INFO')
        if terminal:
            self.ros_logger.info(msg)

    def warn(self, msg, terminal=True):
        self._file(msg, 'WARN')
        if terminal:
            self.ros_logger.warn(msg)

    def error(self, msg, terminal=True):
        self._file(msg, 'ERROR')
        if terminal:
            self.ros_logger.error(msg)

    def detail(self, msg):
        self._file(msg, 'DETAIL')

    def section(self, title):
        sep = '=' * 60
        self._file(sep)
        self._file(f'  {title}', 'SECTION')
        self._file(sep)
        self.ros_logger.info(f'--- {title} ---')

    def csv_row(self, event, mission_id='', input_id='', output_id='',
                material_type='', prod_state='', trans_state='',
                robot_state='', sys_state='', queue_size='',
                weight_kg='', duration_sec='', queue_wait_sec='',
                robot_x='', robot_y='', route_distance='',
                retry_count='', error_msg='', details=''):
        if not self.csv_writer:
            return
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.csv_writer.writerow([
            ts,
            f'{self._elapsed():.2f}',
            event,
            mission_id,
            input_id,
            output_id,
            material_type,
            prod_state,
            trans_state,
            robot_state,
            sys_state,
            queue_size,
            f'{weight_kg:.3f}' if isinstance(weight_kg, float) else weight_kg,
            f'{duration_sec:.2f}' if isinstance(duration_sec, float) else duration_sec,
            f'{queue_wait_sec:.2f}' if isinstance(queue_wait_sec, float) else queue_wait_sec,
            f'{robot_x:.3f}' if isinstance(robot_x, float) else robot_x,
            f'{robot_y:.3f}' if isinstance(robot_y, float) else robot_y,
            f'{route_distance:.3f}' if isinstance(route_distance, float) else route_distance,
            retry_count,
            error_msg,
            details,
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
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None


class Dispatcher(Node):
    def __init__(self):
        super().__init__('dispatcher')

        self.log = DispatcherLogger(self.get_logger())

        # Incarca config
        with open(CONFIG_FILE, 'r') as f:
            self.config = yaml.safe_load(f)
        with open(DOCK_DATABASE_FILE, 'r') as f:
            self.dock_db = yaml.safe_load(f)

        os.makedirs(LOG_DIR, exist_ok=True)

        robot_cfg = self.config['robot']
        disp_cfg  = self.config['dispatcher']

        # Configuratie robot
        self.home_dock_id   = robot_cfg['home_dock_id']
        self.home_dock_type = robot_cfg['home_dock_type']
        self.start_docked   = robot_cfg['start_docked']
        self.auto_dock_home = robot_cfg['auto_dock_home']
        self.max_payload_kg = robot_cfg['max_payload_kg']

        # Configuratie dispatcher
        self.data_gathering_sec       = disp_cfg['data_gathering_sec']
        self.max_consecutive_failures = disp_cfg['max_consecutive_failures']
        self.max_retries              = disp_cfg.get('max_retries', 2)
        self.mission_timeout_sec      = disp_cfg.get(
            'mission_timeout_sec', MISSION_TIMEOUT_SEC
        )

        # Configuratie statii
        self.stations_cfg  = self.config['stations']
        self.material_types = self.config['material_types']

        # ---- Stari ----
        self.prod_state  = PROD_STOPPED
        self.trans_state = TRANS_STOPPED
        self.robot_state = (
            ROBOT_DOCKED_HOME if self.start_docked else ROBOT_IDLE
        )
        self.sys_state = SYS_INITIALIZING

        # ---- Queue ----
        self.queue            = []
        self.current_mission  = None
        self.reserved_outputs = set()

        # ---- Metrici ----
        self.consecutive_failures = 0
        self.session_start        = time.time()
        self.mission_log          = []
        self.total_missions       = 0
        self.successful_missions  = 0
        self.failed_missions      = 0

        # Robot utilization tracking
        self._robot_active_since = None
        self._robot_idle_since   = time.time()
        self.total_robot_active_sec = 0.0
        self.total_robot_idle_sec   = 0.0

        # Route stats
        self.route_stats = defaultdict(list)

        # ---- Robot position ----
        self.robot_x = 0.0
        self.robot_y = 0.0

        home_entry = self.dock_db.get('docks', {}).get(self.home_dock_id, {})
        home_pose  = home_entry.get('pose', [0.0, 0.0, 0.0])
        self.robot_x = home_pose[0]
        self.robot_y = home_pose[1]

        # ---- AMCL tracking ----
        self._amcl_x              = None
        self._amcl_y              = None
        self._amcl_received       = False
        self._robot_state_verified = False

        # ---- Station states cache ----
        self.station_states = {}

        # ---- Publishers ----
        self.status_pub = self.create_publisher(
            String, '/dispatcher/status', 10
        )
        self.goal_pub = self.create_publisher(
            String, '/mission_executor/goal', 10
        )
        self.station_cmd_pub = self.create_publisher(
            String, '/station_manager/cmd', 10
        )

        # ---- Subscribers ----
        self.events_sub = self.create_subscription(
            String, '/station_events', self.on_station_event, 10
        )
        self.states_sub = self.create_subscription(
            String, '/station_states', self.on_station_states, 10
        )
        self.result_sub = self.create_subscription(
            String, '/mission_executor/result', self.on_mission_result, 10
        )
        self.inject_sub = self.create_subscription(
            String, '/dispatcher/inject', self.on_inject, 10
        )
        self.executor_status_sub = self.create_subscription(
            String, '/mission_executor/status', self.on_executor_status, 10
        )
        # AMCL subscriber
        amcl_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.on_amcl_pose,
            amcl_qos,
        )

        # ---- Timere ----
        self.status_timer = self.create_timer(
            1.0 / STATUS_PUBLISH_RATE, self.publish_status
        )
        self.queue_timer = self.create_timer(
            1.0 / QUEUE_CHECK_RATE, self.process_queue
        )

        # ---- Mission log JSONL ----
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.mission_log_path = os.path.join(
            LOG_DIR, f'mission_log_{ts}.jsonl'
        )

        self.log.info(f'Dispatcher pornit. Robot: {self.robot_state}')
        self.log.info(
            f'Config: max_payload={self.max_payload_kg}kg, '
            f'max_retries={self.max_retries}, '
            f'mission_timeout={self.mission_timeout_sec}s, '
            f'max_consecutive_failures={self.max_consecutive_failures}'
        )
        self.log.csv_row(
            'dispatcher_started',
            robot_state=self.robot_state,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            details=(
                f'home={self.home_dock_id}, '
                f'payload_max={self.max_payload_kg}kg'
            ),
        )
        self.update_sys_state()

    # ==============================================================
    # AMCL
    # ==============================================================

    def on_amcl_pose(self, msg):
        self._amcl_x = msg.pose.pose.position.x
        self._amcl_y = msg.pose.pose.position.y

        if not self._amcl_received:
            self._amcl_received = True
            self._verify_robot_state_vs_amcl()

    def _verify_robot_state_vs_amcl(self):
        """
        La prima pozitie AMCL primita verifica daca robotul e
        cu adevarat la home sau nu si corecteaza robot_state.
        """
        if self._robot_state_verified:
            return
        self._robot_state_verified = True

        hx, hy = self.get_dock_position(self.home_dock_id)
        dist = math.sqrt(
            (self._amcl_x - hx) ** 2 + (self._amcl_y - hy) ** 2
        )

        self.log.info(
            f'AMCL check la pornire: robot la '
            f'({self._amcl_x:.2f}, {self._amcl_y:.2f}), '
            f'home la ({hx:.2f}, {hy:.2f}), '
            f'distanta: {dist:.2f}m'
        )
        self.log.csv_row(
            'amcl_startup_check',
            robot_x=self._amcl_x,
            robot_y=self._amcl_y,
            details=f'dist_to_home={dist:.2f}m',
        )

        if self.robot_state == ROBOT_DOCKED_HOME and dist > AMCL_HOME_THRESHOLD:
            self.log.warn(
                f'ATENTIE: config zice start_docked=true dar '
                f'robotul e la {dist:.2f}m de home! '
                f'Resetez la IDLE.'
            )
            self.log.csv_row(
                'robot_state_corrected',
                robot_state='IDLE',
                details=f'was=DOCKED_HOME, dist_to_home={dist:.2f}m',
            )
            self._set_robot_state(ROBOT_IDLE)
            self.robot_x = self._amcl_x
            self.robot_y = self._amcl_y
            self.update_sys_state()

        elif self.robot_state != ROBOT_DOCKED_HOME and dist < 0.3:
            self.log.info(
                f'Robot e la home (dist={dist:.2f}m), confirm DOCKED_HOME.'
            )
            self._set_robot_state(ROBOT_DOCKED_HOME)
            self.robot_x = hx
            self.robot_y = hy
            self.update_sys_state()

        else:
            # Stare corecta, actualizeaza pozitia cu AMCL
            self.robot_x = self._amcl_x
            self.robot_y = self._amcl_y
            self.log.info(
                f'Robot state {self.robot_state} confirmat. '
                f'Pozitie actualizata din AMCL.'
            )

    def _amcl_pre_dispatch_check(self):
        """
        Verifica drift-ul pozitiei inainte de dispatch si
        actualizeaza daca e necesar.
        """
        if not self._amcl_received or self._amcl_x is None:
            return

        dist_drift = math.sqrt(
            (self._amcl_x - self.robot_x) ** 2 +
            (self._amcl_y - self.robot_y) ** 2
        )

        self.log.detail(
            f'Pre-dispatch AMCL: '
            f'estimat=({self.robot_x:.2f},{self.robot_y:.2f}) '
            f'amcl=({self._amcl_x:.2f},{self._amcl_y:.2f}) '
            f'drift={dist_drift:.2f}m'
        )

        if dist_drift > AMCL_DRIFT_THRESHOLD:
            self.log.warn(
                f'Drift pozitie mare pre-dispatch: {dist_drift:.2f}m, '
                f'folosesc AMCL.'
            )
            self.log.csv_row(
                'position_drift_corrected',
                robot_x=self._amcl_x,
                robot_y=self._amcl_y,
                details=f'drift={dist_drift:.2f}m',
            )
            self.robot_x = self._amcl_x
            self.robot_y = self._amcl_y

    # ==============================================================
    # ROBOT STATE + UTILIZATION TRACKING
    # ==============================================================

    def _set_robot_state(self, new_state):
        now = time.time()
        old = self.robot_state

        if old == ROBOT_ACTIVE and self._robot_active_since:
            self.total_robot_active_sec += now - self._robot_active_since
            self._robot_active_since = None
        if old in (ROBOT_IDLE, ROBOT_DOCKED_HOME) and self._robot_idle_since:
            self.total_robot_idle_sec += now - self._robot_idle_since
            self._robot_idle_since = None

        if new_state == ROBOT_ACTIVE:
            self._robot_active_since = now
        elif new_state in (ROBOT_IDLE, ROBOT_DOCKED_HOME):
            self._robot_idle_since = now

        self.robot_state = new_state
        self.log.detail(f'Robot state: {old} -> {new_state}')

    def robot_utilization_percent(self):
        up = time.time() - self.session_start
        if up <= 0:
            return 0.0
        active = self.total_robot_active_sec
        if self._robot_active_since:
            active += time.time() - self._robot_active_since
        return min(100.0, (active / up) * 100.0)

    # ==============================================================
    # SYSTEM STATE
    # ==============================================================

    def update_sys_state(self):
        old = self.sys_state

        if self.robot_state == ROBOT_EMERGENCY_STOP:
            self.sys_state = SYS_EMERGENCY_STOP
        elif (self.prod_state == PROD_PAUSED
              and self.trans_state == TRANS_PAUSED):
            self.sys_state = SYS_PAUSED
        elif (self.prod_state == PROD_RUNNING
              and self.trans_state == TRANS_ACTIVE):
            self.sys_state = SYS_FULL_ACTIVE
        elif self.prod_state == PROD_RUNNING:
            self.sys_state = SYS_PRODUCTION_ONLY
        elif (self.prod_state == PROD_STOPPED
              and self.trans_state == TRANS_STOPPED):
            self.sys_state = SYS_INITIALIZING
        else:
            self.sys_state = SYS_INITIALIZING

        if self.sys_state != old:
            self.log.info(f'Stare sistem: {old} -> {self.sys_state}')
            self.log.csv_row(
                'sys_state_change',
                prod_state=self.prod_state,
                trans_state=self.trans_state,
                robot_state=self.robot_state,
                sys_state=self.sys_state,
                details=f'old={old}',
            )

    # ==============================================================
    # STATION EVENTS
    # ==============================================================

    def on_station_event(self, msg):
        try:
            event = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        ev_type = event.get('event', '')
        sid     = event.get('station_id', '')

        if ev_type == 'station_ready':
            weight = event.get('weight_kg', 0)
            items  = event.get('items', 0)
            fill   = event.get('fill_percent', 0)
            self.log.info(
                f'Station READY: {sid} '
                f'({weight}kg, {items} buc, fill={fill}%)'
            )
            self.log.csv_row(
                'station_ready',
                input_id=sid,
                weight_kg=float(weight),
                queue_size=len(self.queue),
                details=f'items={items}, fill={fill}%',
            )
            self.try_enqueue(sid)

    def on_station_states(self, msg):
        try:
            self.station_states = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    # ==============================================================
    # EXECUTOR STATUS
    # ==============================================================

    def on_executor_status(self, msg):
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        state   = status.get('state', '')
        dock_id = status.get('dock_id', '')

        if not self.current_mission:
            return

        mission_type = self.current_mission.get('type', 'transport')
        if mission_type != 'transport':
            return

        input_dock  = self.current_mission.get('input_dock_id', '')
        output_dock = self.current_mission.get('output_dock_id', '')
        input_id    = self.current_mission.get('input_id', '')
        output_id   = self.current_mission.get('output_id', '')

        if state == 'dwelling' and dock_id == input_dock:
            self.log.info(
                f'Robot la input {input_id}, trimit mark_loading'
            )
            self.send_station_cmd({'cmd': 'mark_loading', 'id': input_id})
            self.log.csv_row(
                'mark_loading_sent',
                mission_id=self.current_mission['mission_id'],
                input_id=input_id,
                details=f'dock={dock_id}',
            )

        if state == 'dwelling' and dock_id == output_dock:
            weight = self.current_mission.get('weight_kg', 0.0)
            self.log.info(
                f'Robot la output {output_id}, '
                f'trimit mark_output_unloading ({weight}kg)'
            )
            self.send_station_cmd({
                'cmd': 'mark_output_unloading',
                'id': output_id,
                'weight_kg': weight,
            })
            self.log.csv_row(
                'mark_unloading_sent',
                mission_id=self.current_mission['mission_id'],
                output_id=output_id,
                weight_kg=weight,
                details=f'dock={dock_id}',
            )

    # ==============================================================
    # RUTARE
    # ==============================================================

    def get_dock_position(self, dock_id):
        entry = self.dock_db.get('docks', {}).get(dock_id, {})
        pose  = entry.get('pose', [0.0, 0.0, 0.0])
        return pose[0], pose[1]

    def get_approach_position(self, dock_id):
        entry = self.dock_db.get('docks', {}).get(dock_id, {})
        ap    = entry.get('approach_point')
        if ap:
            return ap[0], ap[1]
        return self.get_dock_position(dock_id)

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def calculate_route_distance(self, input_dock_id, output_dock_id):
        iax, iay = self.get_approach_position(input_dock_id)
        oax, oay = self.get_approach_position(output_dock_id)
        d_robot_to_input  = self.euclidean_distance(
            self.robot_x, self.robot_y, iax, iay
        )
        d_input_to_output = self.euclidean_distance(iax, iay, oax, oay)
        return d_robot_to_input + d_input_to_output

    def find_best_output(self, input_id):
        input_cfg  = self.stations_cfg.get(input_id, {})
        material   = input_cfg.get('material_type', '')
        compatible = input_cfg.get('compatible_outputs', [])
        input_dock = input_cfg.get('dock_id', input_id)

        candidates    = []
        outputs_state = self.station_states.get('outputs', {})

        for oid in compatible:
            out_cfg = self.stations_cfg.get(oid, {})
            if material not in out_cfg.get('accepted_materials', []):
                continue

            out_state = outputs_state.get(oid, {})
            if out_state.get('status', 'FREE') != 'FREE':
                self.log.detail(
                    f'Output {oid}: skip (status={out_state.get("status")})'
                )
                continue

            if oid in self.reserved_outputs:
                self.log.detail(f'Output {oid}: skip (rezervat)')
                continue

            out_dock   = out_cfg.get('dock_id', oid)
            total_dist = self.calculate_route_distance(input_dock, out_dock)
            candidates.append((oid, total_dist))

        if not candidates:
            return None

        candidates.sort(key=lambda c: c[1])
        best = candidates[0]
        self.log.detail(
            f'Best output pentru {input_id}: {best[0]} '
            f'(dist={best[1]:.2f}m, candidates={len(candidates)})'
        )
        return best[0]

    def try_enqueue(self, input_id):
        if self.trans_state != TRANS_ACTIVE:
            self.log.detail(
                f'try_enqueue({input_id}): skip, '
                f'transport={self.trans_state}'
            )
            return

        for m in self.queue:
            if m['input_id'] == input_id:
                self.log.detail(f'{input_id} deja in queue, skip.')
                return

        input_state = self.station_states.get('inputs', {}).get(input_id, {})
        weight_kg   = input_state.get('current_kg', 0)

        if weight_kg > self.max_payload_kg:
            self.log.warn(
                f'{input_id}: greutate {weight_kg:.2f}kg depaseste '
                f'payload maxim {self.max_payload_kg}kg!'
            )
            self.log.csv_row(
                'payload_exceeded',
                input_id=input_id,
                weight_kg=float(weight_kg),
                details=f'max={self.max_payload_kg}kg',
            )

        output_id = self.find_best_output(input_id)
        if not output_id:
            self.log.info(f'Niciun output disponibil pentru {input_id}.')
            self.log.csv_row(
                'no_output_available',
                input_id=input_id,
                queue_size=len(self.queue),
            )
            return

        input_cfg  = self.stations_cfg.get(input_id, {})
        output_cfg = self.stations_cfg.get(output_id, {})
        mission_id = f'{input_id}_{output_id}_{uuid.uuid4().hex[:6]}'

        input_dock  = input_cfg.get('dock_id', input_id)
        output_dock = output_cfg.get('dock_id', output_id)
        route_dist  = self.calculate_route_distance(input_dock, output_dock)

        now = time.time()
        mission = {
            'mission_id':          mission_id,
            'input_id':            input_id,
            'output_id':           output_id,
            'input_dock_id':       input_dock,
            'output_dock_id':      output_dock,
            'material_type':       input_cfg.get('material_type', ''),
            'priority':            input_cfg.get('priority', 1),
            'load_time':           input_cfg.get('load_time_sec', 5),
            'unload_time':         output_cfg.get('unload_time_sec', 5),
            'weight_kg':           float(weight_kg),
            'route_distance':      route_dist,
            'source':              'auto',
            'queued_at':           now,
            'original_queued_at':  now,   # nu se reseteaza la retry
            'retries':             0,
        }

        self.queue.append(mission)
        self.reserved_outputs.add(output_id)

        self.log.info(
            f'Enqueued: {mission_id} '
            f'({input_id} -> {output_id}, '
            f'{mission["material_type"]}, '
            f'p={mission["priority"]}, '
            f'wt={weight_kg:.2f}kg, '
            f'dist={route_dist:.2f}m)'
        )
        self.log.csv_row(
            'enqueued',
            mission_id=mission_id,
            input_id=input_id,
            output_id=output_id,
            material_type=mission['material_type'],
            queue_size=len(self.queue),
            weight_kg=float(weight_kg),
            route_distance=route_dist,
            details=f'priority={mission["priority"]}',
        )

    # ==============================================================
    # QUEUE SCORING
    # ==============================================================

    def queue_score(self, mission):
        priority = mission.get('priority', 1)
        age      = time.time() - mission.get('queued_at', time.time())

        input_id    = mission.get('input_id', '')
        input_state = self.station_states.get('inputs', {}).get(input_id, {})
        fill_pct    = input_state.get('fill_percent', 0)

        fill_bonus = 50 if fill_pct > 90 else (20 if fill_pct > 75 else 0)
        age_bonus  = min(age / 10.0, 30.0)

        return priority * 100.0 - age_bonus - fill_bonus

    # ==============================================================
    # DISPATCH
    # ==============================================================

    def process_queue(self):
        # Mission timeout check
        if self.current_mission:
            elapsed      = time.time() - self.current_mission.get(
                'dispatched_at', time.time()
            )
            mission_type = self.current_mission.get('type', 'transport')
            if (elapsed > self.mission_timeout_sec
                    and mission_type == 'transport'):
                mid = self.current_mission['mission_id']
                self.log.error(
                    f'TIMEOUT: {mid} dupa {elapsed:.0f}s '
                    f'(max={self.mission_timeout_sec}s)'
                )
                self.log.csv_row(
                    'mission_timeout',
                    mission_id=mid,
                    input_id=self.current_mission.get('input_id', ''),
                    output_id=self.current_mission.get('output_id', ''),
                    duration_sec=elapsed,
                    robot_state=self.robot_state,
                )
                self._handle_mission_failure(
                    self.current_mission,
                    {'error_msg': 'timeout', 'duration_sec': elapsed},
                )
                self.current_mission = None
            return

        if self.trans_state != TRANS_ACTIVE:
            return
        if self.robot_state not in (ROBOT_IDLE, ROBOT_DOCKED_HOME):
            return
        if self.current_mission is not None:
            return
        if not self.queue:
            if self.auto_dock_home and self.robot_state == ROBOT_IDLE:
                self.send_home()
            return

        self.queue.sort(key=lambda m: self.queue_score(m))

        if self.robot_state == ROBOT_DOCKED_HOME:
            self.log.info('Robot docat la home, trimit undock...')
            self.send_undock_home()
            return

        # Re-evaluate output la dispatch time
        mission         = self.queue[0]
        original_output = mission['output_id']
        new_output      = self.find_best_output(mission['input_id'])

        if new_output and new_output != original_output:
            self.reserved_outputs.discard(original_output)
            mission['output_id']      = new_output
            output_cfg                = self.stations_cfg.get(new_output, {})
            mission['output_dock_id'] = output_cfg.get('dock_id', new_output)
            mission['unload_time']    = output_cfg.get('unload_time_sec', 5)
            self.reserved_outputs.add(new_output)
            mission['route_distance'] = self.calculate_route_distance(
                mission['input_dock_id'], mission['output_dock_id']
            )
            self.log.info(
                f'Re-ruted: {mission["mission_id"]} '
                f'{original_output} -> {new_output} '
                f'(dist={mission["route_distance"]:.2f}m)'
            )
            self.log.csv_row(
                'rerouted',
                mission_id=mission['mission_id'],
                input_id=mission['input_id'],
                output_id=new_output,
                route_distance=mission['route_distance'],
                details=f'old_output={original_output}',
            )
        elif not new_output and not self._is_output_available(original_output):
            self.log.warn(
                f'Output {original_output} indisponibil, '
                f'misiune {mission["mission_id"]} ramane in queue.'
            )
            return

        # AMCL pre-dispatch check
        self._amcl_pre_dispatch_check()

        # Pop si dispatch
        self.queue.pop(0)
        output_id = mission['output_id']
        self.reserved_outputs.discard(output_id)

        self.current_mission                = mission
        self.current_mission['dispatched_at'] = time.time()

        queue_wait = (
            self.current_mission['dispatched_at']
            - self.current_mission.get('queued_at', self.current_mission['dispatched_at'])
        )

        self.log.info(
            f'Dispatching: {mission["mission_id"]} '
            f'({mission["input_id"]} -> {mission["output_id"]}, '
            f'wait={queue_wait:.1f}s, '
            f'retry={mission.get("retries", 0)})'
        )

        self._set_robot_state(ROBOT_ACTIVE)
        self.update_sys_state()

        goal = {
            'mission_id':  mission['mission_id'],
            'dock_ids':    [mission['input_dock_id'], mission['output_dock_id']],
            'dwell_times': [mission['load_time'], mission['unload_time']],
            'source':      mission.get('source', 'auto'),
        }

        msg = String()
        msg.data = json.dumps(goal)
        self.goal_pub.publish(msg)

        self.log.info(
            f'Goal trimis: {mission["mission_id"]} '
            f'dock_ids={goal["dock_ids"]}'
        )
        self.log.csv_row(
            'dispatched',
            mission_id=mission['mission_id'],
            input_id=mission['input_id'],
            output_id=mission['output_id'],
            material_type=mission.get('material_type', ''),
            robot_state=self.robot_state,
            queue_size=len(self.queue),
            weight_kg=mission.get('weight_kg', 0.0),
            queue_wait_sec=queue_wait,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            route_distance=mission.get('route_distance', 0.0),
            retry_count=mission.get('retries', 0),
        )

    def _is_output_available(self, output_id):
        out_state = self.station_states.get('outputs', {}).get(output_id, {})
        return out_state.get('status', 'FREE') == 'FREE'

    def send_undock_home(self):
        mission_id = f'undock_home_{uuid.uuid4().hex[:6]}'
        self.current_mission = {
            'mission_id':    mission_id,
            'input_id':      '',
            'output_id':     '',
            'source':        'system',
            'type':          'undock_home',
            'dispatched_at': time.time(),
        }
        self._set_robot_state(ROBOT_ACTIVE)
        self.update_sys_state()

        self.log.info(f'Undock home: {mission_id}')
        self.log.csv_row(
            'undock_home_start',
            mission_id=mission_id,
            robot_state=self.robot_state,
        )
        self._do_undock_home()

    def _do_undock_home(self):
        from rclpy.action import ActionClient as AC
        from nav2_msgs.action import UndockRobot

        if not hasattr(self, '_undock_client'):
            self._undock_client = AC(self, UndockRobot, '/undock_robot')

        if not self._undock_client.wait_for_server(timeout_sec=5.0):
            self.log.error('Undocking server indisponibil.')
            self.log.warn(
                'Undock home esuat (server indisponibil). '
                'Robot trecut in IDLE, misiunea va continua direct.'
            )
            self._set_robot_state(ROBOT_IDLE)
            self.current_mission = None
            self.update_sys_state()
            return

        from nav2_msgs.action import UndockRobot
        goal        = UndockRobot.Goal()
        goal.dock_type = self.home_dock_type

        future = self._undock_client.send_goal_async(goal)

        def on_goal_response(fut):
            handle = fut.result()
            if not handle or not handle.accepted:
                self.log.warn(
                    'Undock home goal respins. '
                    'Robot trecut in IDLE.'
                )
                self.log.csv_row(
                    'undock_home_fail',
                    robot_state='IDLE',
                    details='goal_rejected',
                )
                self._set_robot_state(ROBOT_IDLE)
                self.current_mission = None
                self.update_sys_state()
                return

            result_future = handle.get_result_async()
            result_future.add_done_callback(self._on_undock_home_result)

        future.add_done_callback(on_goal_response)

    def _on_undock_home_result(self, future):
        result = future.result()
        if result and result.result.success:
            self.log.info('Undock home reusit.')
            self.log.csv_row('undock_home_ok', robot_state='IDLE')
            self._set_robot_state(ROBOT_IDLE)
            self.current_mission = None
            self.update_sys_state()
        else:
            # Undock esuat: trece in IDLE, nu in EMERGENCY_STOP
            # Nav2 docking are navigate_to_staging_pose=True,
            # deci se descurca singur la urmatorul dock goal
            self.log.warn(
                'Undock home esuat. '
                'Robot trecut in IDLE, misiunea va continua direct.'
            )
            self.log.csv_row(
                'undock_home_fail',
                robot_state='IDLE',
                details='degraded_to_idle',
            )
            self._set_robot_state(ROBOT_IDLE)
            self.current_mission = None
            self.update_sys_state()

    def send_home(self):
        mission_id = f'go_home_{uuid.uuid4().hex[:6]}'
        self.current_mission = {
            'mission_id':    mission_id,
            'input_id':      '',
            'output_id':     '',
            'source':        'system',
            'type':          'go_home',
            'dispatched_at': time.time(),
        }
        self._set_robot_state(ROBOT_RETURNING_HOME)
        self.update_sys_state()

        goal = {
            'mission_id':  mission_id,
            'dock_ids':    [self.home_dock_id],
            'dwell_times': [0],
            'source':      'system',
        }

        msg      = String()
        msg.data = json.dumps(goal)
        self.goal_pub.publish(msg)

        self.log.info(f'Robot trimis la home: {mission_id}')
        self.log.csv_row(
            'go_home_start',
            mission_id=mission_id,
            robot_state=self.robot_state,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
        )

    # ==============================================================
    # MISSION RESULT
    # ==============================================================

    def on_mission_result(self, msg):
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        mission_id = result.get('mission_id', 'unknown')
        success    = result.get('success', False)
        duration   = result.get('duration_sec', 0)

        self.log.info(
            f'Result: {mission_id} '
            f'{"SUCCESS" if success else "FAILED"} '
            f'({duration}s)'
        )

        if self.current_mission is None:
            self.log.warn('Result primit fara misiune curenta.')
            return

        mission_type = self.current_mission.get('type', 'transport')

        if mission_type == 'go_home':
            if success:
                self._set_robot_state(ROBOT_DOCKED_HOME)
                self.log.info('Robot docat la home.')
                hx, hy = self.get_dock_position(self.home_dock_id)
                self.robot_x = hx
                self.robot_y = hy
            else:
                self._set_robot_state(ROBOT_IDLE)
                self.log.warn('Go home esuat, robot IDLE.')
            self.log.csv_row(
                'go_home_result',
                mission_id=mission_id,
                robot_state=self.robot_state,
                duration_sec=float(duration),
                robot_x=self.robot_x,
                robot_y=self.robot_y,
                details='success' if success else 'failed',
            )
            self.current_mission = None
            self.update_sys_state()
            return

        queue_wait = (
            self.current_mission.get('dispatched_at', 0)
            - self.current_mission.get('queued_at', 0)
        )

        if success:
            self._handle_mission_success(self.current_mission, result)
        else:
            self._handle_mission_failure(self.current_mission, result)

        mission_data = {
            **self.current_mission,
            'success':      success,
            'duration_sec': duration,
            'completed_at': time.time(),
            'metrics':      result.get('metrics', {}),
            'error_msg':    result.get('error_msg', ''),
            'queue_wait_sec': queue_wait,
        }
        self.log_mission(mission_data)
        self.mission_log.append(mission_data)

        self.current_mission = None
        self.update_sys_state()

    def _handle_mission_success(self, mission, result):
        mission_id = mission['mission_id']
        input_id   = mission.get('input_id', '')
        output_id  = mission.get('output_id', '')
        duration   = result.get('duration_sec', 0)
        queue_wait = (
            mission.get('dispatched_at', 0) - mission.get('queued_at', 0)
        )

        self.consecutive_failures = 0
        self.successful_missions += 1
        self.total_missions      += 1

        self.send_station_cmd({'cmd': 'mark_pickup_complete', 'id': input_id})
        self.send_station_cmd({'cmd': 'mark_output_cleared',  'id': output_id})

        ox, oy = self.get_dock_position(
            mission.get('output_dock_id', output_id)
        )
        self.robot_x = ox
        self.robot_y = oy

        self._set_robot_state(ROBOT_IDLE)

        route = f'{input_id}->{output_id}'
        self.route_stats[route].append(float(duration))

        self.log.info(
            f'Mission OK: {mission_id} '
            f'({input_id}->{output_id}, '
            f'{duration}s, wait={queue_wait:.1f}s)'
        )
        self.log.csv_row(
            'mission_success',
            mission_id=mission_id,
            input_id=input_id,
            output_id=output_id,
            material_type=mission.get('material_type', ''),
            robot_state=self.robot_state,
            queue_size=len(self.queue),
            weight_kg=mission.get('weight_kg', 0.0),
            duration_sec=float(duration),
            queue_wait_sec=queue_wait,
            robot_x=self.robot_x,
            robot_y=self.robot_y,
            route_distance=mission.get('route_distance', 0.0),
            retry_count=mission.get('retries', 0),
        )

    def _handle_mission_failure(self, mission, result):
        mission_id = mission['mission_id']
        input_id   = mission.get('input_id', '')
        output_id  = mission.get('output_id', '')
        duration   = result.get('duration_sec', 0)
        error_msg  = result.get('error_msg', 'unknown')
        retries    = mission.get('retries', 0)

        self.consecutive_failures += 1

        completed = result.get('completed_docks', [])
        if completed:
            last_dock  = completed[-1]
            lx, ly     = self.get_dock_position(last_dock)
            self.robot_x = lx
            self.robot_y = ly
            self.log.detail(
                f'Pozitie robot update din completed_docks: '
                f'{last_dock} ({lx:.3f}, {ly:.3f})'
            )

        self.log.error(
            f'Mission FAIL: {mission_id} '
            f'({input_id}->{output_id}, '
            f'err={error_msg}, '
            f'retry={retries}/{self.max_retries}, '
            f'consecutive_fail={self.consecutive_failures})'
        )

        if retries < self.max_retries:
            mission['retries']      = retries + 1
            mission['retry_reason'] = error_msg
            mission['queued_at']    = time.time()  # reset scoring
            # original_queued_at ramane neschimbat
            self.queue.insert(0, mission)
            self.reserved_outputs.add(output_id)

            self.log.info(f'Retry {retries+1}/{self.max_retries}: {mission_id}')
            self.log.csv_row(
                'mission_retry',
                mission_id=mission_id,
                input_id=input_id,
                output_id=output_id,
                duration_sec=float(duration),
                retry_count=retries + 1,
                error_msg=error_msg,
                robot_x=self.robot_x,
                robot_y=self.robot_y,
            )
            self._set_robot_state(ROBOT_IDLE)
        else:
            self.failed_missions += 1
            self.total_missions  += 1

            self.log.csv_row(
                'mission_failed',
                mission_id=mission_id,
                input_id=input_id,
                output_id=output_id,
                material_type=mission.get('material_type', ''),
                duration_sec=float(duration),
                retry_count=retries,
                error_msg=error_msg,
                robot_x=self.robot_x,
                robot_y=self.robot_y,
            )

            if self.consecutive_failures >= self.max_consecutive_failures:
                self.log.error(
                    'EMERGENCY STOP: prea multe esecuri consecutive '
                    f'({self.consecutive_failures})'
                )
                self.log.csv_row(
                    'emergency_stop',
                    robot_state='EMERGENCY_STOP',
                    details=f'consecutive_failures={self.consecutive_failures}',
                )
                self._set_robot_state(ROBOT_EMERGENCY_STOP)
            else:
                self._set_robot_state(ROBOT_IDLE)

    # ==============================================================
    # INJECT
    # ==============================================================

    def on_inject(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.log.error('JSON invalid pe /dispatcher/inject')
            return

        cmd_type = cmd.get('type', '')
        self.log.info(f'Inject: {cmd_type} {cmd}')
        self.log.csv_row(
            f'inject_{cmd_type}',
            details=json.dumps(cmd),
        )

        if cmd_type == 'start_production':
            reason = cmd.get('reason', '')
            self.prod_state = PROD_RUNNING
            self.send_station_cmd({'cmd': 'start_all'})
            msg_str = 'Productia pornita'
            if reason:
                msg_str += f' (reason: {reason})'
            self.log.info(msg_str)
            self.log.csv_row(
                'start_production',
                prod_state=self.prod_state,
                details=f'reason={reason}' if reason else '',
            )

        elif cmd_type == 'stop_production':
            reason = cmd.get('reason', 'operator_stop')
            self.prod_state = PROD_STOPPED
            self.send_station_cmd({'cmd': 'stop_all', 'reason': reason})
            self.log.info(f'Productia oprita (reason: {reason})')
            self.log.csv_row(
                'stop_production',
                prod_state=self.prod_state,
                details=f'reason={reason}',
            )

        elif cmd_type == 'start_transport':
            self.trans_state = TRANS_ACTIVE
            self.log.info('Transportul pornit.')
            self._check_existing_ready()

        elif cmd_type == 'stop_transport':
            self.trans_state = TRANS_STOPPED
            self.log.info(
                'Transportul oprit. Robot termina misiunea curenta.'
            )

        elif cmd_type == 'pause':
            self.prod_state  = PROD_PAUSED
            self.trans_state = TRANS_PAUSED
            self.log.info('Sistem in pauza.')

        elif cmd_type == 'resume':
            self.prod_state  = PROD_RUNNING
            self.trans_state = TRANS_ACTIVE
            self.send_station_cmd({'cmd': 'start_all'})
            self.log.info('Sistem reluat.')
            self._check_existing_ready()

        elif cmd_type == 'abort':
            self.log.error('ABORT: oprire imediata.')
            self.prod_state  = PROD_STOPPED
            self.trans_state = TRANS_STOPPED
            self.queue.clear()
            self.reserved_outputs.clear()
            self.send_station_cmd({'cmd': 'stop_all'})
            self.generate_report()

        elif cmd_type == 'status':
            self.log.info(
                f'Status: prod={self.prod_state} '
                f'trans={self.trans_state} '
                f'robot={self.robot_state} '
                f'sys={self.sys_state} '
                f'queue={len(self.queue)} '
                f'missions={self.total_missions} '
                f'ok={self.successful_missions} '
                f'fail={self.failed_missions} '
                f'utilization={self.robot_utilization_percent():.1f}%'
            )

        elif cmd_type == 'skip':
            if self.current_mission:
                mid       = self.current_mission['mission_id']
                input_id  = self.current_mission.get('input_id', '')
                output_id = self.current_mission.get('output_id', '')
                self.reserved_outputs.discard(output_id)
                self.log.warn(f'SKIP: {mid} ({input_id}->{output_id})')
                self.log.csv_row(
                    'mission_skipped',
                    mission_id=mid,
                    input_id=input_id,
                    output_id=output_id,
                    details='operator_skip',
                )
                self.current_mission = None
                self._set_robot_state(ROBOT_IDLE)
                self.update_sys_state()
            else:
                self.log.warn('Skip: nicio misiune curenta.')

        elif cmd_type == 'priority':
            station = cmd.get('station', '')
            self._set_priority(station)

        elif cmd_type == 'manual':
            input_id  = cmd.get('input', '')
            output_id = cmd.get('output', '')
            self._manual_mission(input_id, output_id)

        elif cmd_type == 'simulate':
            station = cmd.get('station', '')
            self.send_station_cmd({'cmd': 'simulate', 'id': station})

        elif cmd_type == 'stop_station':
            self.send_station_cmd({
                'cmd':    'stop_station',
                'id':     cmd.get('station', ''),
                'reason': cmd.get('reason', 'operator_stop'),
            })

        elif cmd_type == 'start_station':
            self.send_station_cmd({
                'cmd': 'start_station',
                'id':  cmd.get('station', ''),
            })

        elif cmd_type == 'set_fault':
            self.send_station_cmd({
                'cmd':    'set_fault',
                'id':     cmd.get('station', ''),
                'reason': cmd.get('reason', 'fault'),
            })

        elif cmd_type == 'clear_station':
            self.send_station_cmd({
                'cmd': 'clear_station',
                'id':  cmd.get('station', ''),
            })

        elif cmd_type == 'stop_output':
            self.send_station_cmd({
                'cmd':    'stop_output',
                'id':     cmd.get('output', ''),
                'reason': cmd.get('reason', 'operator_stop'),
            })

        elif cmd_type == 'start_output':
            self.send_station_cmd({
                'cmd': 'start_output',
                'id':  cmd.get('output', ''),
            })

        elif cmd_type == 'set_output_full':
            self.send_station_cmd({
                'cmd': 'set_output_full',
                'id':  cmd.get('output', ''),
            })

        elif cmd_type == 'clear_output':
            self.send_station_cmd({
                'cmd': 'clear_output',
                'id':  cmd.get('output', ''),
            })

        elif cmd_type == 'clear_emergency':
            if self.robot_state == ROBOT_EMERGENCY_STOP:
                self.consecutive_failures = 0
                self._set_robot_state(ROBOT_IDLE)
                self.log.info('Emergency stop cleared.')
            else:
                self.log.warn(
                    f'clear_emergency: robot nu e in EMERGENCY_STOP '
                    f'(state={self.robot_state})'
                )

        elif cmd_type == 'report':
            self.generate_report()

        else:
            self.log.warn(f'Inject necunoscut: {cmd_type}')

        self.update_sys_state()
        self.save_snapshot(cmd)

    def _set_priority(self, station_id):
        for i, m in enumerate(self.queue):
            if m['input_id'] == station_id:
                mission = self.queue.pop(i)
                mission['priority'] = 0
                self.queue.insert(0, mission)
                self.log.info(f'Priority boost: {station_id}')
                return
        self.log.warn(f'{station_id} nu e in queue pentru priority.')

    def _manual_mission(self, input_id, output_id):
        if input_id not in self.stations_cfg:
            self.log.warn(f'Input necunoscut: {input_id}')
            return
        if output_id not in self.stations_cfg:
            self.log.warn(f'Output necunoscut: {output_id}')
            return

        input_cfg  = self.stations_cfg[input_id]
        output_cfg = self.stations_cfg[output_id]
        mission_id = f'manual_{input_id}_{output_id}_{uuid.uuid4().hex[:6]}'

        input_dock  = input_cfg.get('dock_id', input_id)
        output_dock = output_cfg.get('dock_id', output_id)
        route_dist  = self.calculate_route_distance(input_dock, output_dock)

        now = time.time()
        mission = {
            'mission_id':         mission_id,
            'input_id':           input_id,
            'output_id':          output_id,
            'input_dock_id':      input_dock,
            'output_dock_id':     output_dock,
            'material_type':      input_cfg.get('material_type', ''),
            'priority':           0,
            'load_time':          input_cfg.get('load_time_sec', 5),
            'unload_time':        output_cfg.get('unload_time_sec', 5),
            'weight_kg':          0.0,
            'route_distance':     route_dist,
            'source':             'manual',
            'queued_at':          now,
            'original_queued_at': now,
            'retries':            0,
        }

        self.queue.insert(0, mission)
        self.reserved_outputs.add(output_id)
        self.log.info(f'Manual mission: {mission_id}')
        self.log.csv_row(
            'manual_enqueued',
            mission_id=mission_id,
            input_id=input_id,
            output_id=output_id,
            queue_size=len(self.queue),
            route_distance=route_dist,
        )

    def _check_existing_ready(self):
        inputs = self.station_states.get('inputs', {})
        for sid, state in inputs.items():
            if state.get('status') == 'READY':
                self.log.detail(
                    f'Statie deja READY la start transport: {sid}'
                )
                self.try_enqueue(sid)

    # ==============================================================
    # STATION COMMANDS
    # ==============================================================

    def send_station_cmd(self, cmd_dict):
        msg      = String()
        msg.data = json.dumps(cmd_dict)
        self.station_cmd_pub.publish(msg)
        self.log.detail(f'Station CMD: {cmd_dict}')

    # ==============================================================
    # STATUS PUBLISH
    # ==============================================================

    def publish_status(self):
        queue_info = []
        for m in self.queue:
            queue_info.append({
                'mission_id': m['mission_id'],
                'input':      m['input_id'],
                'output':     m['output_id'],
                'priority':   m['priority'],
                'weight_kg':  m.get('weight_kg', 0),
                'retries':    m.get('retries', 0),
                # queue_age_sec din original_queued_at, nu se reseteaza la retry
                'queue_age_sec': round(
                    time.time() - m.get(
                        'original_queued_at',
                        m.get('queued_at', time.time())
                    ), 1
                ),
            })

        current = None
        if self.current_mission:
            elapsed = time.time() - self.current_mission.get(
                'dispatched_at', time.time()
            )
            current = {
                'mission_id': self.current_mission['mission_id'],
                'input':      self.current_mission.get('input_id', ''),
                'output':     self.current_mission.get('output_id', ''),
                'type':       self.current_mission.get('type', 'transport'),
                'elapsed_sec': round(elapsed, 1),
                'retries':    self.current_mission.get('retries', 0),
            }

        status = {
            'prod_state':    self.prod_state,
            'trans_state':   self.trans_state,
            'robot_state':   self.robot_state,
            'sys_state':     self.sys_state,
            'queue':         queue_info,
            'queue_size':    len(self.queue),
            'current_mission': current,
            'robot_pos': {
                'x': round(self.robot_x, 3),
                'y': round(self.robot_y, 3),
            },
            'amcl_pos': {
                'x': round(self._amcl_x, 3) if self._amcl_x is not None else None,
                'y': round(self._amcl_y, 3) if self._amcl_y is not None else None,
            },
            'total_missions':       self.total_missions,
            'successful':           self.successful_missions,
            'failed':               self.failed_missions,
            'consecutive_failures': self.consecutive_failures,
            'robot_utilization_pct': round(
                self.robot_utilization_percent(), 1
            ),
            'uptime_sec': round(time.time() - self.session_start, 1),
            'timestamp':  time.time(),
        }

        msg      = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    # ==============================================================
    # SNAPSHOT
    # ==============================================================

    def save_snapshot(self, trigger_cmd):
        snap_dir = os.path.join(LOG_DIR, 'snapshots')
        os.makedirs(snap_dir, exist_ok=True)
        ts   = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        snap = {
            'timestamp':    time.time(),
            'trigger':      trigger_cmd,
            'prod_state':   self.prod_state,
            'trans_state':  self.trans_state,
            'robot_state':  self.robot_state,
            'sys_state':    self.sys_state,
            'queue_size':   len(self.queue),
            'total_missions':      self.total_missions,
            'successful':          self.successful_missions,
            'failed':              self.failed_missions,
            'robot_utilization_pct': round(
                self.robot_utilization_percent(), 1
            ),
            'station_states': self.station_states,
        }
        path = os.path.join(snap_dir, f'snapshot_{ts}.json')
        with open(path, 'w') as f:
            json.dump(snap, f, indent=2)

    # ==============================================================
    # MISSION LOG
    # ==============================================================

    def log_mission(self, mission_data):
        with open(self.mission_log_path, 'a') as f:
            f.write(json.dumps(mission_data) + '\n')

    # ==============================================================
    # RAPORT
    # ==============================================================

    def generate_report(self):
        ts          = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_path = os.path.join(LOG_DIR, f'session_report_{ts}.txt')
        data_path   = os.path.join(LOG_DIR, f'session_data_{ts}.json')

        uptime       = time.time() - self.session_start
        success_rate = (
            (self.successful_missions / self.total_missions * 100)
            if self.total_missions > 0 else 0
        )

        total_weight = sum(
            m.get('weight_kg', 0)
            for m in self.mission_log if m.get('success')
        )
        total_duration_missions = sum(
            m.get('duration_sec', 0) for m in self.mission_log
        )
        missions_per_hour = (
            (self.total_missions / (uptime / 3600)) if uptime > 0 else 0
        )

        queue_waits = [
            m.get('queue_wait_sec', 0)
            for m in self.mission_log if m.get('success')
        ]
        avg_queue_wait = (
            sum(queue_waits) / len(queue_waits) if queue_waits else 0.0
        )

        robot_util = self.robot_utilization_percent()

        lines = [
            '=' * 60,
            '  NOUZEN SESSION REPORT',
            f'  Generated: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}',
            '=' * 60,
            '',
            f'Session duration:       {uptime:.0f}s ({uptime/60:.1f} min)',
            f'Total missions:         {self.total_missions}',
            f'Successful:             {self.successful_missions}',
            f'Failed:                 {self.failed_missions}',
            f'Success rate:           {success_rate:.1f}%',
            f'Missions/hour:          {missions_per_hour:.1f}',
            f'Total weight (est):     {total_weight:.2f} kg',
            f'Total mission time:     {total_duration_missions:.1f}s',
            f'Avg queue wait:         {avg_queue_wait:.1f}s',
            f'Robot utilization:      {robot_util:.1f}%',
            '',
        ]

        if self.route_stats:
            lines.append('--- Route Statistics ---')
            for route, durations in sorted(self.route_stats.items()):
                count = len(durations)
                avg   = sum(durations) / count
                mn    = min(durations)
                mx    = max(durations)
                lines.append(
                    f'  {route}: n={count}, '
                    f'avg={avg:.1f}s, min={mn:.1f}s, max={mx:.1f}s'
                )
            lines.append('')

        lines.append('--- Mission Details ---')
        for m in self.mission_log:
            status_str = 'OK' if m.get('success') else 'FAIL'
            retry      = m.get('retries', 0)
            retry_str  = f' retry={retry}' if retry > 0 else ''
            orig_wait  = m.get('original_queued_at', 0)
            disp_time  = m.get('dispatched_at', 0)
            total_wait = (disp_time - orig_wait) if orig_wait and disp_time else 0
            lines.append(
                f'  {m.get("mission_id", "?")} '
                f'[{status_str}] '
                f'{m.get("input_id", "")}->{m.get("output_id", "")} '
                f'{m.get("duration_sec", 0):.1f}s '
                f'total_wait={total_wait:.1f}s'
                f'{retry_str} '
                f'{m.get("error_msg", "")}'
            )

        lines.append('')
        lines.append('=' * 60)

        report_text = '\n'.join(lines)

        with open(report_path, 'w') as f:
            f.write(report_text)

        with open(data_path, 'w') as f:
            json.dump({
                'session_start':    self.session_start,
                'session_end':      time.time(),
                'uptime_sec':       uptime,
                'total_missions':   self.total_missions,
                'successful':       self.successful_missions,
                'failed':           self.failed_missions,
                'success_rate':     success_rate,
                'robot_utilization_pct': robot_util,
                'avg_queue_wait_sec':    avg_queue_wait,
                'route_stats': {
                    route: {
                        'count':   len(durs),
                        'avg_sec': sum(durs) / len(durs),
                        'min_sec': min(durs),
                        'max_sec': max(durs),
                    }
                    for route, durs in self.route_stats.items()
                },
                'missions': self.mission_log,
            }, f, indent=2)

        self.log.info(f'Raport generat: {report_path}')
        self.log.section('SESSION REPORT')
        for line in lines:
            self.log.info(line, terminal=False)
        self.get_logger().info(f'\n{report_text}')

    # ==============================================================
    # CLEANUP
    # ==============================================================

    def cleanup(self):
        self.generate_report()
        self.log.close()


def main():
    rclpy.init()
    node = Dispatcher()

    def signal_handler(sig, frame):
        node.log.info('SIGINT primit, generez raport...')
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        try:
            node.cleanup()
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()