#!/usr/bin/env python3
"""
station_manager.py
Nod ROS2 care gestioneaza toate statiile (input + output).
Simuleaza productia continua, publica evenimente si stari.

Topicuri publicate:
  /station_events   std_msgs/String JSON
  /station_states   std_msgs/String JSON

Topicuri ascultate:
  /station_manager/cmd  std_msgs/String JSON
"""

import csv
import json
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import yaml

from station import (
    InputStation, OutputStation,
    INPUT_IDLE, INPUT_PRODUCING, INPUT_READY,
    INPUT_LOADING, INPUT_RESTOCKING,
    INPUT_STOPPED, INPUT_FAULT, INPUT_NO_MATERIAL,
    OUTPUT_FREE, OUTPUT_UNLOADING, OUTPUT_CLEARING,
    OUTPUT_STOPPED, OUTPUT_FAULT, OUTPUT_FULL,
)

CONFIG_FILE = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/config/station_config.yaml'
)
LOG_DIR = os.path.expanduser(
    '~/saim_nouzen/src/amr2ax_nav2/logs'
)

TICK_RATE = 1.0  # Hz


class StationLogger:
    """Dual .log + .csv logger pentru station_manager."""

    def __init__(self, ros_logger, log_dir=None):
        self.ros_logger = ros_logger
        self.start_time = time.time()

        log_dir = log_dir or LOG_DIR
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # .log file
        log_path = os.path.join(log_dir, f'station_manager_{ts}.log')
        self.log_file = open(log_path, 'w')
        self._file('station_manager pornit')
        self._file(f'Log: {log_path}')
        self._file('=' * 60)
        self.ros_logger.info(f'Log: {log_path}')

        # .csv file
        csv_path = os.path.join(log_dir, f'station_manager_{ts}.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'elapsed_sec',
            'station_id',
            'station_type',       # input / output
            'event',              # batch_produced, station_ready, cmd_*, etc
            'old_status',
            'new_status',
            'weight_kg',
            'items',
            'fill_percent',
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

    def csv_row(self, station_id, station_type, event,
                old_status='', new_status='',
                weight_kg='', items='', fill_percent='',
                details=''):
        if not self.csv_writer:
            return
        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        self.csv_writer.writerow([
            ts,
            f'{self._elapsed():.2f}',
            station_id,
            station_type,
            event,
            old_status,
            new_status,
            f'{weight_kg:.3f}' if isinstance(weight_kg, float) else weight_kg,
            items,
            f'{fill_percent:.1f}' if isinstance(fill_percent, float) else fill_percent,
            details,
        ])
        self.csv_file.flush()

    def section(self, title):
        sep = '=' * 60
        self._file(sep)
        self._file(f'  {title}', 'SECTION')
        self._file(sep)

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


class StationManager(Node):
    def __init__(self):
        super().__init__('station_manager')

        self.log = StationLogger(self.get_logger())

        # Incarca config
        with open(CONFIG_FILE, 'r') as f:
            self.config = yaml.safe_load(f)

        material_types = self.config['material_types']
        stations_cfg = self.config['stations']

        # Creeaza statii
        self.inputs = {}
        self.outputs = {}

        for sid, cfg in stations_cfg.items():
            if cfg['type'] == 'input':
                mat_cfg = material_types[cfg['material_type']]
                self.inputs[sid] = InputStation(sid, cfg, mat_cfg)
                self.log.info(
                    f'Input creat: {sid} (material={cfg["material_type"]}, '
                    f'dock={cfg["dock_id"]}, '
                    f'min={cfg["min_box_capacity_kg"]}kg, '
                    f'max={cfg["max_box_capacity_kg"]}kg)'
                )
                self.log.csv_row(
                    sid, 'input', 'station_created',
                    new_status=INPUT_IDLE,
                    details=f'material={cfg["material_type"]}'
                )
            elif cfg['type'] == 'output':
                self.outputs[sid] = OutputStation(sid, cfg)
                self.log.info(
                    f'Output creat: {sid} (dock={cfg["dock_id"]}, '
                    f'accepted={cfg["accepted_materials"]})'
                )
                self.log.csv_row(
                    sid, 'output', 'station_created',
                    new_status=OUTPUT_FREE,
                    details=f'accepted={cfg["accepted_materials"]}'
                )

        self.log.info(
            f'Statii incarcate: {len(self.inputs)} input, '
            f'{len(self.outputs)} output'
        )

        # Publishers
        self.events_pub = self.create_publisher(String, '/station_events', 10)
        self.states_pub = self.create_publisher(String, '/station_states', 10)

        # Subscriber comenzi
        self.cmd_sub = self.create_subscription(
            String, '/station_manager/cmd', self.on_cmd, 10
        )

        # Timer tick
        self.tick_timer = self.create_timer(1.0 / TICK_RATE, self.tick)

        self._states_dirty = True
        self.log.info('station_manager pornit.')

    def publish_event(self, event_dict):
        msg = String()
        msg.data = json.dumps(event_dict)
        self.events_pub.publish(msg)
        self.log.detail(
            f'Event pub: {event_dict["event"]} '
            f'station={event_dict.get("station_id", "")}'
        )

    def publish_states(self):
        state = {
            'inputs': {sid: s.to_dict() for sid, s in self.inputs.items()},
            'outputs': {sid: s.to_dict() for sid, s in self.outputs.items()},
            'timestamp': time.time(),
        }
        msg = String()
        msg.data = json.dumps(state)
        self.states_pub.publish(msg)

    def tick(self):
        any_events = False

        for station in self.inputs.values():
            old_status = station.status
            events = station.tick()
            for ev in events:
                self.publish_event(ev)
                any_events = True

                # CSV per eveniment
                self.log.csv_row(
                    station.id, 'input', ev['event'],
                    old_status=old_status,
                    new_status=station.status,
                    weight_kg=ev.get('weight_kg', ''),
                    items=ev.get('items', ''),
                    fill_percent=station.fill_percent(),
                )

        for station in self.outputs.values():
            old_status = station.status
            events = station.tick()
            for ev in events:
                self.publish_event(ev)
                any_events = True

                self.log.csv_row(
                    station.id, 'output', ev['event'],
                    old_status=old_status,
                    new_status=station.status,
                    weight_kg=ev.get('cleared_kg', ''),
                )

        if any_events or self._states_dirty:
            self.publish_states()
            self._states_dirty = False

    def on_cmd(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError:
            self.log.error(f'JSON invalid pe /station_manager/cmd: {msg.data}')
            return

        cmd_type = cmd.get('cmd', '')
        sid = cmd.get('id', '')

        self.log.info(f'CMD: {cmd_type} id={sid}')

        if cmd_type == 'start_all':
            for s in self.inputs.values():
                old = s.status
                ok = s.start()
                if ok:
                    self.log.csv_row(
                        s.id, 'input', 'cmd_start_all',
                        old_status=old, new_status=s.status,
                    )
            self._states_dirty = True

        elif cmd_type == 'stop_all':
            reason = cmd.get('reason', 'operator_stop')
            for s in self.inputs.values():
                old = s.status
                ok = s.stop(reason)
                if ok:
                    self.log.csv_row(
                        s.id, 'input', 'cmd_stop_all',
                        old_status=old, new_status=s.status,
                        details=f'reason={reason}',
                    )
            self._states_dirty = True

        elif cmd_type == 'start_station':
            station = self.inputs.get(sid)
            if station:
                old = station.status
                station.start()
                self.log.csv_row(
                    sid, 'input', 'cmd_start_station',
                    old_status=old, new_status=station.status,
                )
                self._states_dirty = True
            else:
                self.log.warn(f'Statie input necunoscuta: {sid}')

        elif cmd_type == 'stop_station':
            station = self.inputs.get(sid)
            if station:
                old = station.status
                reason = cmd.get('reason', 'operator_stop')
                station.stop(reason)
                self.publish_event({
                    'event': 'station_stopped',
                    'station_id': sid,
                    'reason': reason,
                })
                self.log.csv_row(
                    sid, 'input', 'cmd_stop_station',
                    old_status=old, new_status=station.status,
                    details=f'reason={reason}',
                )
                self._states_dirty = True

        elif cmd_type == 'set_fault':
            station = self.inputs.get(sid)
            if station:
                old = station.status
                reason = cmd.get('reason', 'fault')
                station.set_fault(reason)
                self.publish_event({
                    'event': 'station_fault',
                    'station_id': sid,
                    'reason': reason,
                })
                self.log.csv_row(
                    sid, 'input', 'cmd_set_fault',
                    old_status=old, new_status=station.status,
                    details=f'reason={reason}',
                )
                self._states_dirty = True

        elif cmd_type == 'clear_station':
            station = self.inputs.get(sid)
            if station:
                old = station.status
                station.clear_fault()
                self.publish_event({
                    'event': 'station_cleared',
                    'station_id': sid,
                })
                self.log.csv_row(
                    sid, 'input', 'cmd_clear_station',
                    old_status=old, new_status=station.status,
                )
                self._states_dirty = True

        elif cmd_type == 'simulate':
            station = self.inputs.get(sid)
            if station:
                old = station.status
                kg, items = station.simulate_fill()
                self.publish_event({
                    'event': 'station_ready',
                    'station_id': sid,
                    'weight_kg': round(kg, 3),
                    'items': items,
                    'fill_percent': round(station.fill_percent(), 1),
                })
                self.log.csv_row(
                    sid, 'input', 'cmd_simulate',
                    old_status=old, new_status=station.status,
                    weight_kg=kg, items=items,
                    fill_percent=station.fill_percent(),
                )
                self._states_dirty = True

        elif cmd_type == 'mark_loading':
            station = self.inputs.get(sid)
            if station:
                old = station.status
                ok = station.mark_loading()
                if ok:
                    self.publish_event({
                        'event': 'station_loading',
                        'station_id': sid,
                    })
                    self.log.csv_row(
                        sid, 'input', 'cmd_mark_loading',
                        old_status=old, new_status=station.status,
                        weight_kg=station.current_kg,
                        items=station.current_items,
                    )
                else:
                    self.log.warn(
                        f'mark_loading esuat pe {sid} '
                        f'(status={station.status})'
                    )
                self._states_dirty = True

        elif cmd_type == 'mark_pickup_complete':
            station = self.inputs.get(sid)
            if station:
                old = station.status
                picked_kg, picked_items = station.mark_pickup_complete()
                self.publish_event({
                    'event': 'station_restocking',
                    'station_id': sid,
                    'picked_kg': round(picked_kg, 3),
                    'picked_items': picked_items,
                })
                self.log.info(
                    f'Pickup complete: {sid} '
                    f'{picked_kg:.3f}kg, {picked_items} buc '
                    f'(total pickups: {station.total_pickups})'
                )
                self.log.csv_row(
                    sid, 'input', 'cmd_mark_pickup_complete',
                    old_status=old, new_status=station.status,
                    weight_kg=picked_kg, items=picked_items,
                    details=f'total_pickups={station.total_pickups}',
                )
                self._states_dirty = True

        elif cmd_type == 'stop_output':
            station = self.outputs.get(sid)
            if station:
                old = station.status
                reason = cmd.get('reason', 'operator_stop')
                station.stop(reason)
                self.publish_event({
                    'event': 'output_stopped',
                    'station_id': sid,
                    'reason': reason,
                })
                self.log.csv_row(
                    sid, 'output', 'cmd_stop_output',
                    old_status=old, new_status=station.status,
                    details=f'reason={reason}',
                )
                self._states_dirty = True

        elif cmd_type == 'start_output':
            station = self.outputs.get(sid)
            if station:
                old = station.status
                station.clear()
                self.publish_event({
                    'event': 'output_cleared',
                    'station_id': sid,
                })
                self.log.csv_row(
                    sid, 'output', 'cmd_start_output',
                    old_status=old, new_status=station.status,
                )
                self._states_dirty = True

        elif cmd_type == 'set_output_full':
            station = self.outputs.get(sid)
            if station:
                old = station.status
                station.set_full()
                self.publish_event({
                    'event': 'output_full',
                    'station_id': sid,
                })
                self.log.csv_row(
                    sid, 'output', 'cmd_set_output_full',
                    old_status=old, new_status=station.status,
                )
                self._states_dirty = True

        elif cmd_type == 'clear_output':
            station = self.outputs.get(sid)
            if station:
                old = station.status
                station.clear()
                self.publish_event({
                    'event': 'output_cleared',
                    'station_id': sid,
                })
                self.log.csv_row(
                    sid, 'output', 'cmd_clear_output',
                    old_status=old, new_status=station.status,
                )
                self._states_dirty = True

        elif cmd_type == 'mark_output_unloading':
            station = self.outputs.get(sid)
            if station:
                old = station.status
                weight = cmd.get('weight_kg', 0.0)
                ok = station.mark_unloading(weight)
                if ok:
                    self.publish_event({
                        'event': 'output_unloading',
                        'station_id': sid,
                        'weight_kg': weight,
                    })
                    self.log.csv_row(
                        sid, 'output', 'cmd_mark_output_unloading',
                        old_status=old, new_status=station.status,
                        weight_kg=weight,
                    )
                else:
                    self.log.warn(
                        f'mark_unloading esuat pe {sid} '
                        f'(status={station.status})'
                    )
                self._states_dirty = True

        elif cmd_type == 'mark_output_cleared':
            station = self.outputs.get(sid)
            if station:
                old = station.status
                station.mark_unload_complete()
                self.log.info(
                    f'Unload complete: {sid} '
                    f'(total deliveries: {station.total_deliveries})'
                )
                self.log.csv_row(
                    sid, 'output', 'cmd_mark_output_cleared',
                    old_status=old, new_status=station.status,
                    weight_kg=station.current_kg,
                    details=f'total_deliveries={station.total_deliveries}',
                )
                self._states_dirty = True

        else:
            self.log.warn(f'Comanda necunoscuta: {cmd_type}')

    def generate_report(self):
        """Raport final cu metrici agregate per statie."""
        self.log.section('STATION MANAGER REPORT')

        for sid, s in self.inputs.items():
            d = s.to_dict()
            m = d['metrics']
            self.log.info(
                f'INPUT {sid}: pickups={m["total_pickups"]}, '
                f'weight_out={m["total_weight_transported_kg"]}kg, '
                f'items_out={m["total_items_transported"]}, '
                f'batches={m["total_batches_produced"]}, '
                f'producing={m["producing_percent"]}%, '
                f'stopped={m["total_stopped_time_sec"]}s, '
                f'fault={m["total_fault_time_sec"]}s'
            )

        for sid, s in self.outputs.items():
            d = s.to_dict()
            m = d['metrics']
            self.log.info(
                f'OUTPUT {sid}: deliveries={m["total_deliveries"]}, '
                f'weight_in={m["total_weight_received_kg"]}kg, '
                f'cleared={m["total_cleared_kg"]}kg, '
                f'utilization={m["utilization_percent"]}%, '
                f'stopped={m["total_stopped_time_sec"]}s, '
                f'fault={m["total_fault_time_sec"]}s'
            )


def main():
    rclpy.init()
    node = StationManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.generate_report()
        node.log.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()