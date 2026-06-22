#!/usr/bin/env python3
"""
inject.py
Script CLI pentru comenzi operator catre dispatcher.
Publica pe /dispatcher/inject (std_msgs/String JSON).

Folosire:
  python3 inject.py start_production
  python3 inject.py start_transport
  python3 inject.py stop_production
  python3 inject.py stop_transport
  python3 inject.py pause
  python3 inject.py resume
  python3 inject.py abort
  python3 inject.py status
  python3 inject.py report
  python3 inject.py clear_emergency
  python3 inject.py priority <input_id>
  python3 inject.py manual <input_id> <output_id>
  python3 inject.py simulate <input_id>
  python3 inject.py stop_station <input_id> [reason]
  python3 inject.py start_station <input_id>
  python3 inject.py set_fault <input_id> [reason]
  python3 inject.py clear_station <input_id>
  python3 inject.py stop_output <output_id> [reason]
  python3 inject.py start_output <output_id>
  python3 inject.py set_output_full <output_id>
  python3 inject.py clear_output <output_id>
"""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


VALID_REASONS = [
    'operator_stop', 'no_material', 'broken',
    'maintenance', 'full', 'fault',
]

USAGE = """
NOUZEN Inject - Comenzi operator

Control sistem:
  start_production [reason]     Porneste productia (reason optional)
  stop_production [reason]      Opreste productia (default: operator_stop)
  start_transport               Porneste transportul (dispatcher dispatch-uieste)
  stop_transport                Opreste transportul (termina misiunea curenta)
  pause                         Pauza totala (productie + transport)
  resume                        Reia totul
  abort                         Oprire imediata + generare raport
  status                        Afiseaza starea curenta
  report                        Genereaza raport sesiune on-demand
  clear_emergency               Resetare EMERGENCY_STOP + consecutive failures
  skip                          Skip misiunea curenta (daca exista)

Control misiuni:
  priority <input>          Muta misiunea unui input in fata queue-ului
  manual <input> <output>   Adauga misiune manuala cu prioritate maxima
  simulate <input>          Umple instant un input (DEMO)

Control statii INPUT:
  stop_station <input> [reason]    Opreste o statie input
  start_station <input>            Porneste o statie input
  set_fault <input> [reason]       Seteaza defectiune pe input
  clear_station <input>            Elibereaza input din FAULT/STOPPED

Control statii OUTPUT:
  stop_output <output> [reason]    Opreste o statie output
  start_output <output>            Porneste o statie output
  set_output_full <output>         Marcheaza output ca FULL
  clear_output <output>            Elibereaza output

Motive valide pentru reason:
  operator_stop, no_material, broken, maintenance, full, fault
"""


def build_command(args):
    """Construieste dict-ul JSON din argumentele CLI."""
    if not args:
        return None

    cmd = args[0]

    # Comenzi fara parametri
    if cmd in ('start_transport', 'stop_transport',
               'pause', 'resume', 'abort', 'status',
               'report', 'clear_emergency'):
        return {'type': cmd}

    # priority <input>
    if cmd == 'priority':
        if len(args) < 2:
            print('Eroare: priority necesita <input_id>')
            return None
        return {'type': 'priority', 'station': args[1]}

    # manual <input> <output>
    if cmd == 'manual':
        if len(args) < 3:
            print('Eroare: manual necesita <input_id> <output_id>')
            return None
        return {'type': 'manual', 'input': args[1], 'output': args[2]}

    # simulate <input>
    if cmd == 'simulate':
        if len(args) < 2:
            print('Eroare: simulate necesita <input_id>')
            return None
        return {'type': 'simulate', 'station': args[1]}

    # stop_station <input> [reason]
    if cmd == 'stop_station':
        if len(args) < 2:
            print('Eroare: stop_station necesita <input_id>')
            return None
        reason = args[2] if len(args) >= 3 else 'operator_stop'
        return {'type': 'stop_station', 'station': args[1], 'reason': reason}

    # start_station <input>
    if cmd == 'start_station':
        if len(args) < 2:
            print('Eroare: start_station necesita <input_id>')
            return None
        return {'type': 'start_station', 'station': args[1]}

    # set_fault <input> [reason]
    if cmd == 'set_fault':
        if len(args) < 2:
            print('Eroare: set_fault necesita <input_id>')
            return None
        reason = args[2] if len(args) >= 3 else 'fault'
        return {'type': 'set_fault', 'station': args[1], 'reason': reason}

    # clear_station <input>
    if cmd == 'clear_station':
        if len(args) < 2:
            print('Eroare: clear_station necesita <input_id>')
            return None
        return {'type': 'clear_station', 'station': args[1]}

    # stop_output <output> [reason]
    if cmd == 'stop_output':
        if len(args) < 2:
            print('Eroare: stop_output necesita <output_id>')
            return None
        reason = args[2] if len(args) >= 3 else 'operator_stop'
        return {'type': 'stop_output', 'output': args[1], 'reason': reason}

    # start_output <output>
    if cmd == 'start_output':
        if len(args) < 2:
            print('Eroare: start_output necesita <output_id>')
            return None
        return {'type': 'start_output', 'output': args[1]}

    # set_output_full <output>
    if cmd == 'set_output_full':
        if len(args) < 2:
            print('Eroare: set_output_full necesita <output_id>')
            return None
        return {'type': 'set_output_full', 'output': args[1]}

    # clear_output <output>
    if cmd == 'clear_output':
        if len(args) < 2:
            print('Eroare: clear_output necesita <output_id>')
            return None
        return {'type': 'clear_output', 'output': args[1]}

    # skip (nicio misiune curenta)
    if cmd == 'skip':
        return {'type': 'skip'}

    # start_production [reason]
    if cmd == 'start_production':
        reason = args[1] if len(args) >= 2 else ''
        d = {'type': 'start_production'}
        if reason:
            d['reason'] = reason
        return d

    # stop_production [reason]
    if cmd == 'stop_production':
        reason = args[1] if len(args) >= 2 else 'operator_stop'
        return {'type': 'stop_production', 'reason': reason}

    print(f'Comanda necunoscuta: {cmd}')
    return None


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ('-h', '--help', 'help'):
        print(USAGE)
        return

    command = build_command(sys.argv[1:])
    if command is None:
        sys.exit(1)

    rclpy.init()
    node = rclpy.create_node('inject_cmd')
    pub = node.create_publisher(String, '/dispatcher/inject', 10)

    # Asteapta sa se conecteze publisher-ul
    time.sleep(0.3)

    msg = String()
    msg.data = json.dumps(command)
    pub.publish(msg)

    # Spin scurt ca sa se trimita mesajul
    time.sleep(0.1)
    for _ in range(3):
        rclpy.spin_once(node, timeout_sec=0.05)

    cmd_str = sys.argv[1]
    args_str = ' '.join(sys.argv[2:]) if len(sys.argv) > 2 else ''
    print(f'[inject] {cmd_str} {args_str} -> trimis')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()