#!/usr/bin/env python3
"""
station.py
Clasele InputStation si OutputStation.
Logica de productie simulata, stari, umplere progresiva.
Importat de station_manager.py.
"""

import time


# ============================================================
# STARI
# ============================================================
# Input
INPUT_IDLE        = 'IDLE'
INPUT_PRODUCING   = 'PRODUCING'
INPUT_READY       = 'READY'
INPUT_LOADING     = 'LOADING'
INPUT_RESTOCKING  = 'RESTOCKING'
INPUT_STOPPED     = 'STOPPED'
INPUT_FAULT       = 'FAULT'
INPUT_NO_MATERIAL = 'NO_MATERIAL'

# Output
OUTPUT_FREE      = 'FREE'
OUTPUT_UNLOADING = 'UNLOADING'
OUTPUT_CLEARING  = 'CLEARING'
OUTPUT_STOPPED   = 'STOPPED'
OUTPUT_FAULT     = 'FAULT'
OUTPUT_FULL      = 'FULL'


class InputStation:
    def __init__(self, station_id, cfg, material_cfg):
        self.id = station_id
        self.dock_id = cfg['dock_id']
        self.tag_id = cfg['tag_id']
        self.approach_yaw = cfg['approach_yaw']
        self.material_type = cfg['material_type']
        self.min_kg = cfg['min_box_capacity_kg']
        self.max_kg = cfg['max_box_capacity_kg']
        self.load_time_sec = cfg['load_time_sec']
        self.restock_delay_sec = cfg['restock_delay_sec']
        self.priority = cfg.get('priority', 1)
        self.compatible_outputs = cfg['compatible_outputs']

        # Material
        self.weight_per_item = material_cfg['weight_kg']
        self.items_per_batch = material_cfg['items_per_batch']
        self.production_interval = material_cfg['production_interval_sec']

        # Stare
        self.status = INPUT_IDLE
        self.prev_status = INPUT_IDLE
        self.current_kg = 0.0
        self.current_items = 0
        self.last_produce_time = 0.0
        self.restock_start_time = 0.0
        self.stop_reason = ''
        self.last_status_change = time.time()

        # Metrici
        self.created_at = time.time()
        self.total_pickups = 0
        self.total_weight_transported_kg = 0.0
        self.total_items_transported = 0
        self.total_batches_produced = 0
        self.total_items_produced = 0
        self.total_stopped_time_sec = 0.0
        self.total_fault_time_sec = 0.0
        self.total_producing_time_sec = 0.0
        self._stopped_since = None
        self._fault_since = None
        self._producing_since = None

    def _set_status(self, new_status):
        """Helper: seteaza statusul si actualizeaza timere de tracking."""
        now = time.time()
        old = self.status

        # Inchide timer-ul starii vechi
        if old == INPUT_PRODUCING and self._producing_since:
            self.total_producing_time_sec += now - self._producing_since
            self._producing_since = None
        if old in (INPUT_STOPPED, INPUT_NO_MATERIAL) and self._stopped_since:
            self.total_stopped_time_sec += now - self._stopped_since
            self._stopped_since = None
        if old == INPUT_FAULT and self._fault_since:
            self.total_fault_time_sec += now - self._fault_since
            self._fault_since = None

        # Deschide timer-ul starii noi
        if new_status == INPUT_PRODUCING:
            self._producing_since = now
        elif new_status in (INPUT_STOPPED, INPUT_NO_MATERIAL):
            self._stopped_since = now
        elif new_status == INPUT_FAULT:
            self._fault_since = now

        self.prev_status = old
        self.status = new_status
        self.last_status_change = now

    def start(self):
        """Porneste productia."""
        if self.status in (INPUT_IDLE, INPUT_STOPPED, INPUT_NO_MATERIAL):
            self._set_status(INPUT_PRODUCING)
            self.last_produce_time = time.time()
            self.stop_reason = ''
            return True
        return False

    def stop(self, reason='operator_stop'):
        """Opreste statia."""
        if self.status in (INPUT_PRODUCING, INPUT_READY, INPUT_IDLE):
            self._set_status(INPUT_STOPPED)
            self.stop_reason = reason
            return True
        return False

    def set_fault(self, reason='fault'):
        """Seteaza defectiune."""
        self._set_status(INPUT_FAULT)
        self.stop_reason = reason
        return True

    def set_no_material(self):
        """Lipsa materie prima."""
        self._set_status(INPUT_NO_MATERIAL)
        self.stop_reason = 'no_material'
        return True

    def clear_fault(self):
        """Elibereaza din FAULT/STOPPED/NO_MATERIAL, revine la PRODUCING."""
        if self.status in (INPUT_FAULT, INPUT_STOPPED, INPUT_NO_MATERIAL):
            self._set_status(INPUT_PRODUCING)
            self.stop_reason = ''
            self.last_produce_time = time.time()
            return True
        return False

    def tick(self):
        """
        Apelat periodic de station_manager.
        Returneaza lista de evenimente generate.
        """
        events = []
        now = time.time()

        if self.status == INPUT_PRODUCING:
            elapsed = now - self.last_produce_time
            if elapsed >= self.production_interval:
                batch_weight = self.weight_per_item * self.items_per_batch
                new_kg = self.current_kg + batch_weight
                if new_kg <= self.max_kg:
                    self.current_kg = new_kg
                    self.current_items += self.items_per_batch
                    self.total_batches_produced += 1
                    self.total_items_produced += self.items_per_batch
                    self.last_produce_time = now
                    events.append({
                        'event': 'batch_produced',
                        'station_id': self.id,
                        'weight_kg': round(self.current_kg, 3),
                        'items': self.current_items,
                        'batch_num': self.total_batches_produced,
                    })

                    if self.current_kg >= self.min_kg and self.status == INPUT_PRODUCING:
                        self._set_status(INPUT_READY)
                        events.append({
                            'event': 'station_ready',
                            'station_id': self.id,
                            'weight_kg': round(self.current_kg, 3),
                            'items': self.current_items,
                            'fill_percent': round(self.fill_percent(), 1),
                        })
                else:
                    if self.status != INPUT_READY:
                        self._set_status(INPUT_READY)
                        events.append({
                            'event': 'station_ready',
                            'station_id': self.id,
                            'weight_kg': round(self.current_kg, 3),
                            'items': self.current_items,
                            'fill_percent': round(self.fill_percent(), 1),
                        })

        elif self.status == INPUT_READY:
            elapsed = now - self.last_produce_time
            if elapsed >= self.production_interval:
                batch_weight = self.weight_per_item * self.items_per_batch
                new_kg = self.current_kg + batch_weight
                if new_kg <= self.max_kg:
                    self.current_kg = new_kg
                    self.current_items += self.items_per_batch
                    self.total_batches_produced += 1
                    self.total_items_produced += self.items_per_batch
                    self.last_produce_time = now
                    events.append({
                        'event': 'batch_produced',
                        'station_id': self.id,
                        'weight_kg': round(self.current_kg, 3),
                        'items': self.current_items,
                        'batch_num': self.total_batches_produced,
                    })

        elif self.status == INPUT_RESTOCKING:
            elapsed = now - self.restock_start_time
            if elapsed >= self.restock_delay_sec:
                self._set_status(INPUT_PRODUCING)
                self.last_produce_time = now
                events.append({
                    'event': 'station_producing',
                    'station_id': self.id,
                })

        return events

    def mark_loading(self):
        """Robotul a ajuns si incepe sa incarce."""
        if self.status in (INPUT_READY, INPUT_PRODUCING):
            self._set_status(INPUT_LOADING)
            return True
        return False

    def mark_pickup_complete(self):
        """Robotul a terminat de incarcat. Reseteaza cutia."""
        if self.status == INPUT_LOADING:
            picked_kg = self.current_kg
            picked_items = self.current_items
            self.total_pickups += 1
            self.total_weight_transported_kg += picked_kg
            self.total_items_transported += picked_items
            self.current_kg = 0.0
            self.current_items = 0
            self._set_status(INPUT_RESTOCKING)
            self.restock_start_time = time.time()
            return picked_kg, picked_items
        return 0.0, 0

    def simulate_fill(self):
        """Umple instant la max_kg (comanda demo)."""
        items_to_fill = int(self.max_kg / self.weight_per_item)
        self.current_kg = items_to_fill * self.weight_per_item
        self.current_items = items_to_fill
        self._set_status(INPUT_READY)
        return self.current_kg, self.current_items

    def fill_percent(self):
        if self.max_kg <= 0:
            return 0.0
        return min(100.0, (self.current_kg / self.max_kg) * 100.0)

    def uptime_sec(self):
        return time.time() - self.created_at

    def producing_percent(self):
        """Cat % din uptime a fost in PRODUCING/READY."""
        up = self.uptime_sec()
        if up <= 0:
            return 0.0
        prod = self.total_producing_time_sec
        if self._producing_since:
            prod += time.time() - self._producing_since
        return min(100.0, (prod / up) * 100.0)

    def to_dict(self):
        return {
            'id': self.id,
            'type': 'input',
            'status': self.status,
            'prev_status': self.prev_status,
            'material_type': self.material_type,
            'current_kg': round(self.current_kg, 3),
            'current_items': self.current_items,
            'min_kg': self.min_kg,
            'max_kg': self.max_kg,
            'fill_percent': round(self.fill_percent(), 1),
            'priority': self.priority,
            'stop_reason': self.stop_reason,
            'dock_id': self.dock_id,
            'compatible_outputs': self.compatible_outputs,
            'metrics': {
                'total_pickups': self.total_pickups,
                'total_weight_transported_kg': round(self.total_weight_transported_kg, 3),
                'total_items_transported': self.total_items_transported,
                'total_batches_produced': self.total_batches_produced,
                'total_items_produced': self.total_items_produced,
                'total_stopped_time_sec': round(self.total_stopped_time_sec, 1),
                'total_fault_time_sec': round(self.total_fault_time_sec, 1),
                'producing_percent': round(self.producing_percent(), 1),
                'uptime_sec': round(self.uptime_sec(), 1),
            },
        }


class OutputStation:
    def __init__(self, station_id, cfg):
        self.id = station_id
        self.dock_id = cfg['dock_id']
        self.tag_id = cfg['tag_id']
        self.approach_yaw = cfg['approach_yaw']
        self.accepted_materials = cfg['accepted_materials']
        self.unload_time_sec = cfg['unload_time_sec']
        self.clear_time_sec = cfg['clear_time_sec']

        # Stare
        self.status = OUTPUT_FREE
        self.prev_status = OUTPUT_FREE
        self.current_kg = 0.0
        self.stop_reason = ''
        self.clear_start_time = 0.0
        self.last_status_change = time.time()

        # Metrici
        self.created_at = time.time()
        self.total_deliveries = 0
        self.total_weight_received_kg = 0.0
        self.total_cleared_kg = 0.0
        self.total_stopped_time_sec = 0.0
        self.total_fault_time_sec = 0.0
        self.total_busy_time_sec = 0.0  # UNLOADING + CLEARING
        self._stopped_since = None
        self._fault_since = None
        self._busy_since = None

    def _set_status(self, new_status):
        now = time.time()
        old = self.status

        if old in (OUTPUT_STOPPED,) and self._stopped_since:
            self.total_stopped_time_sec += now - self._stopped_since
            self._stopped_since = None
        if old == OUTPUT_FAULT and self._fault_since:
            self.total_fault_time_sec += now - self._fault_since
            self._fault_since = None
        if old in (OUTPUT_UNLOADING, OUTPUT_CLEARING) and self._busy_since:
            self.total_busy_time_sec += now - self._busy_since
            self._busy_since = None

        if new_status in (OUTPUT_STOPPED, OUTPUT_FULL):
            self._stopped_since = now
        elif new_status == OUTPUT_FAULT:
            self._fault_since = now
        elif new_status in (OUTPUT_UNLOADING, OUTPUT_CLEARING):
            if not self._busy_since:
                self._busy_since = now

        self.prev_status = old
        self.status = new_status
        self.last_status_change = now

    def tick(self):
        """Apelat periodic. Gestioneaza clearing."""
        events = []
        now = time.time()

        if self.status == OUTPUT_CLEARING:
            elapsed = now - self.clear_start_time
            if elapsed >= self.clear_time_sec:
                cleared_kg = self.current_kg
                self.total_cleared_kg += cleared_kg
                self._set_status(OUTPUT_FREE)
                self.current_kg = 0.0
                events.append({
                    'event': 'output_cleared',
                    'station_id': self.id,
                    'cleared_kg': round(cleared_kg, 3),
                })

        return events

    def is_available(self):
        return self.status == OUTPUT_FREE

    def accepts_material(self, material_type):
        return material_type in self.accepted_materials

    def mark_unloading(self, weight_kg):
        """Robotul a ajuns si descarca."""
        if self.status == OUTPUT_FREE:
            self._set_status(OUTPUT_UNLOADING)
            self.current_kg = weight_kg
            return True
        return False

    def mark_unload_complete(self):
        """Robotul a terminat descarcarea. Incepe clearing."""
        if self.status == OUTPUT_UNLOADING:
            self.total_deliveries += 1
            self.total_weight_received_kg += self.current_kg
            self._set_status(OUTPUT_CLEARING)
            self.clear_start_time = time.time()
            return True
        return False

    def stop(self, reason='operator_stop'):
        if self.status in (OUTPUT_FREE, OUTPUT_CLEARING):
            self._set_status(OUTPUT_STOPPED)
            self.stop_reason = reason
            return True
        return False

    def set_full(self):
        self._set_status(OUTPUT_FULL)
        self.stop_reason = 'full'
        return True

    def set_fault(self, reason='fault'):
        self._set_status(OUTPUT_FAULT)
        self.stop_reason = reason
        return True

    def clear(self):
        """Elibereaza din orice stare blocata."""
        if self.status in (OUTPUT_STOPPED, OUTPUT_FULL, OUTPUT_FAULT):
            self._set_status(OUTPUT_FREE)
            self.current_kg = 0.0
            self.stop_reason = ''
            return True
        return False

    def uptime_sec(self):
        return time.time() - self.created_at

    def utilization_percent(self):
        """Cat % din uptime a fost busy (UNLOADING+CLEARING)."""
        up = self.uptime_sec()
        if up <= 0:
            return 0.0
        busy = self.total_busy_time_sec
        if self._busy_since:
            busy += time.time() - self._busy_since
        return min(100.0, (busy / up) * 100.0)

    def to_dict(self):
        return {
            'id': self.id,
            'type': 'output',
            'status': self.status,
            'prev_status': self.prev_status,
            'accepted_materials': self.accepted_materials,
            'current_kg': round(self.current_kg, 3),
            'stop_reason': self.stop_reason,
            'dock_id': self.dock_id,
            'metrics': {
                'total_deliveries': self.total_deliveries,
                'total_weight_received_kg': round(self.total_weight_received_kg, 3),
                'total_cleared_kg': round(self.total_cleared_kg, 3),
                'total_stopped_time_sec': round(self.total_stopped_time_sec, 1),
                'total_fault_time_sec': round(self.total_fault_time_sec, 1),
                'utilization_percent': round(self.utilization_percent(), 1),
                'uptime_sec': round(self.uptime_sec(), 1),
            },
        }