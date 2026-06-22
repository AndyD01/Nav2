#!/usr/bin/env python3
"""
dashboard.py
Dashboard terminal curses pentru NOUZEN.
Asculta /station_states si /dispatcher/status, update event-driven.

Layout:
  +--HEADER (sys state, uptime, clock)-------------------------+
  | INPUT STATIONS        | ROBOT & MISSION                    |
  |                       | QUEUE                              |
  +--separator------------+ METRICS                            |
  | OUTPUT STATIONS       |                                    |
  +--CMD BAR (keybinds)--+------------------------------------+
  +--FOOTER----------------------------------------------+

Keybinds:
  1  start_production    2  stop_production
  3  start_transport     4  stop_transport
  5  skip mission        6  report
  R  resume              P  pause
  A  abort               E  clear_emergency
  Q  quit

Pornire:
  python3 dashboard.py
"""

import curses
import json
import threading
import time
from datetime import timedelta

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# ============================================================
# CULORI
# ============================================================
COLOR_DEFAULT = 0
COLOR_GREEN   = 1
COLOR_YELLOW  = 2
COLOR_RED     = 3
COLOR_CYAN    = 4
COLOR_MAGENTA = 5
COLOR_WHITE   = 6
COLOR_HEADER  = 7
COLOR_DIM     = 8
COLOR_BLUE    = 9

STATUS_COLORS = {
    # Input
    'IDLE':        COLOR_DIM,
    'PRODUCING':   COLOR_YELLOW,
    'READY':       COLOR_GREEN,
    'LOADING':     COLOR_CYAN,
    'RESTOCKING':  COLOR_YELLOW,
    'STOPPED':     COLOR_RED,
    'FAULT':       COLOR_RED,
    'NO_MATERIAL': COLOR_RED,
    # Output
    'FREE':        COLOR_GREEN,
    'UNLOADING':   COLOR_CYAN,
    'CLEARING':    COLOR_YELLOW,
    'FULL':        COLOR_RED,
    # Robot
    'DOCKED_HOME':    COLOR_GREEN,
    'IDLE':           COLOR_DEFAULT,
    'ACTIVE':         COLOR_CYAN,
    'RETURNING_HOME': COLOR_YELLOW,
    'RETRY':          COLOR_MAGENTA,
    'EMERGENCY_STOP': COLOR_RED,
    'PAUSED':         COLOR_MAGENTA,
    # Productie / Transport
    'RUNNING':    COLOR_GREEN,
    'ACTIVE':     COLOR_GREEN,
    # Sistem
    'INITIALIZING':    COLOR_DIM,
    'PRODUCTION_ONLY': COLOR_YELLOW,
    'FULL_ACTIVE':     COLOR_GREEN,
    'SHUTTING_DOWN':   COLOR_RED,
    'EMERGENCY_STOP':  COLOR_RED,
    'PAUSED':          COLOR_MAGENTA,
}

# Box drawing
BOX_H  = '\u2500'
BOX_V  = '\u2502'
BOX_TL = '\u250c'
BOX_TR = '\u2510'
BOX_BL = '\u2514'
BOX_BR = '\u2518'
BOX_LJ = '\u251c'
BOX_RJ = '\u2524'
BOX_TJ = '\u252c'
BOX_BJ = '\u2534'
BOX_CJ = '\u253c'

# Progress bar
BAR_FULL  = '\u2588'
BAR_EMPTY = '\u2591'

# Keybinds -> (label afisata, comanda inject)
KEYBINDS = [
    ('1', 'start prod',   {'type': 'start_production'}),
    ('2', 'stop prod',    {'type': 'stop_production', 'reason': 'dashboard'}),
    ('3', 'start trans',  {'type': 'start_transport'}),
    ('4', 'stop trans',   {'type': 'stop_transport'}),
    ('5', 'skip',         {'type': 'skip'}),
    ('6', 'report',       {'type': 'report'}),
    ('R', 'resume',       {'type': 'resume'}),
    ('P', 'pause',        {'type': 'pause'}),
    ('A', 'abort',        {'type': 'abort'}),
    ('E', 'clr emrg',     {'type': 'clear_emergency'}),
]


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard')

        self.station_data    = {}
        self.dispatcher_data = {}
        self.lock            = threading.Lock()

        # Ultimul feedback de comanda (afisat temporar)
        self._last_cmd_feedback    = ''
        self._last_cmd_feedback_ts = 0.0

        self.create_subscription(
            String, '/station_states', self.on_station_states, 10
        )
        self.create_subscription(
            String, '/dispatcher/status', self.on_dispatcher_status, 10
        )
        self.inject_pub = self.create_publisher(
            String, '/dispatcher/inject', 10
        )

    def on_station_states(self, msg):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.station_data = data
        except json.JSONDecodeError:
            pass

    def on_dispatcher_status(self, msg):
        try:
            data = json.loads(msg.data)
            with self.lock:
                self.dispatcher_data = data
        except json.JSONDecodeError:
            pass

    def get_snapshot(self):
        with self.lock:
            return dict(self.station_data), dict(self.dispatcher_data)

    def send_inject(self, cmd_dict):
        msg      = String()
        msg.data = json.dumps(cmd_dict)
        self.inject_pub.publish(msg)
        label = cmd_dict.get('type', '?')
        with self.lock:
            self._last_cmd_feedback    = f'Sent: {label}'
            self._last_cmd_feedback_ts = time.time()

    def get_cmd_feedback(self):
        with self.lock:
            if time.time() - self._last_cmd_feedback_ts < 2.0:
                return self._last_cmd_feedback
            return ''


# ============================================================
# HELPERS
# ============================================================

def init_colors():
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(COLOR_GREEN,   curses.COLOR_GREEN,   -1)
    curses.init_pair(COLOR_YELLOW,  curses.COLOR_YELLOW,  -1)
    curses.init_pair(COLOR_RED,     curses.COLOR_RED,     -1)
    curses.init_pair(COLOR_CYAN,    curses.COLOR_CYAN,    -1)
    curses.init_pair(COLOR_MAGENTA, curses.COLOR_MAGENTA, -1)
    curses.init_pair(COLOR_WHITE,   curses.COLOR_WHITE,   -1)
    curses.init_pair(COLOR_HEADER,  curses.COLOR_BLACK,   curses.COLOR_CYAN)
    curses.init_pair(COLOR_DIM,     curses.COLOR_WHITE,   -1)
    curses.init_pair(COLOR_BLUE,    curses.COLOR_BLUE,    -1)


def safe_addstr(win, y, x, text, attr=0):
    h, w = win.getmaxyx()
    if y >= h or x >= w:
        return
    max_len = w - x - 1
    if max_len <= 0:
        return
    win.addnstr(y, x, str(text), max_len, attr)


def draw_hline(win, y, x1, x2, char=BOX_H, attr=0):
    h, w = win.getmaxyx()
    if y >= h:
        return
    for cx in range(x1, min(x2, w - 1)):
        try:
            win.addstr(y, cx, char, attr)
        except curses.error:
            pass


def draw_vline(win, y1, y2, x, char=BOX_V, attr=0):
    h, w = win.getmaxyx()
    if x >= w:
        return
    for cy in range(y1, min(y2, h)):
        try:
            win.addstr(cy, x, char, attr)
        except curses.error:
            pass


def draw_progress_bar(win, y, x, width, percent):
    h, w = win.getmaxyx()
    if y >= h or x >= w or width < 2:
        return
    inner = min(width, w - x - 1)
    if inner < 1:
        return
    filled = max(0, min(int(inner * percent / 100.0), inner))

    if percent >= 90:
        bar_color = curses.color_pair(COLOR_GREEN) | curses.A_BOLD
    elif percent >= 50:
        bar_color = curses.color_pair(COLOR_YELLOW)
    elif percent > 0:
        bar_color = curses.color_pair(COLOR_DIM)
    else:
        bar_color = curses.color_pair(COLOR_DIM) | curses.A_DIM

    bar_str = BAR_FULL * filled + BAR_EMPTY * (inner - filled)
    safe_addstr(win, y, x, bar_str, bar_color)


def colored_status(win, y, x, status, bold=True):
    color = curses.color_pair(STATUS_COLORS.get(status, COLOR_DEFAULT))
    attr  = color | (curses.A_BOLD if bold else 0)
    safe_addstr(win, y, x, status, attr)
    return len(status)


def format_uptime(seconds):
    td      = timedelta(seconds=int(seconds))
    hours, r = divmod(td.seconds, 3600)
    minutes, secs = divmod(r, 60)
    if td.days > 0:
        return f'{td.days}d {hours:02d}:{minutes:02d}:{secs:02d}'
    return f'{hours:02d}:{minutes:02d}:{secs:02d}'


# ============================================================
# MAIN DRAW LOOP
# ============================================================

def draw_dashboard(stdscr, node):
    init_colors()
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(300)

    bold_attr   = curses.A_BOLD
    dim_attr    = curses.color_pair(COLOR_DIM) | curses.A_DIM
    header_attr = curses.color_pair(COLOR_HEADER) | curses.A_BOLD

    while True:
        key = stdscr.getch()

        # ---- Keybinds ----
        if key in (ord('q'), ord('Q')):
            break
        for kb_key, _, kb_cmd in KEYBINDS:
            if key == ord(kb_key) or key == ord(kb_key.lower()):
                node.send_inject(kb_cmd)
                break

        station_data, disp_data = node.get_snapshot()
        stdscr.erase()
        h, w = stdscr.getmaxyx()

        if h < 20 or w < 100:
            safe_addstr(stdscr, 0, 0,
                        f'Terminal prea mic ({w}x{h}). Minim: 100x20')
            stdscr.refresh()
            continue

        # Rezervam randuri pentru cmd bar si footer
        CMD_BAR_Y = h - 3
        FOOTER_Y  = h - 1
        CONTENT_H = CMD_BAR_Y - 1  # randuri disponibile pentru continut

        # Divisor vertical intre stanga si dreapta
        col_div  = w // 2
        col1_x   = 1
        col2_x   = col_div + 2
        col1_w   = col_div - 2
        col2_w   = w - col_div - 3

        # ============================================================
        # HEADER (2 randuri)
        # ============================================================
        now_str   = time.strftime('%H:%M:%S')
        uptime    = format_uptime(disp_data.get('uptime_sec', 0))
        sys_state = disp_data.get('sys_state', 'UNKNOWN')
        prod      = disp_data.get('prod_state',  'STOPPED')
        trans     = disp_data.get('trans_state', 'STOPPED')
        robot_state = disp_data.get('robot_state', 'IDLE')

        # Rand 0: title bar
        title  = ' NOUZEN AMR INTRALOGISTIC '
        rinfo  = f' {now_str}  Up:{uptime} '
        pad    = max(0, w - 1 - len(title) - len(rinfo))
        h_line = title + ' ' * pad + rinfo
        safe_addstr(stdscr, 0, 0, h_line.ljust(w - 1), header_attr)

        # Rand 1: stari sistem inline
        cx = 1
        safe_addstr(stdscr, 1, cx, 'SYS:', bold_attr)
        cx += 5
        colored_status(stdscr, 1, cx, sys_state)
        cx += len(sys_state) + 2

        safe_addstr(stdscr, 1, cx, 'PROD:', bold_attr)
        cx += 6
        colored_status(stdscr, 1, cx, prod)
        cx += len(prod) + 2

        safe_addstr(stdscr, 1, cx, 'TRANS:', bold_attr)
        cx += 7
        colored_status(stdscr, 1, cx, trans)
        cx += len(trans) + 2

        safe_addstr(stdscr, 1, cx, 'ROBOT:', bold_attr)
        cx += 7
        colored_status(stdscr, 1, cx, robot_state)
        cx += len(robot_state) + 2

        # AMCL pos pe acelasi rand, aliniat dreapta
        amcl = disp_data.get('amcl_pos', {})
        if amcl and amcl.get('x') is not None:
            amcl_str = (
                f'AMCL:({amcl["x"]:.2f},{amcl["y"]:.2f}) '
            )
            safe_addstr(stdscr, 1, w - len(amcl_str) - 1,
                        amcl_str, dim_attr)

        # Separator header
        draw_hline(stdscr, 2, 0, w, BOX_H, dim_attr)

        # Divisor vertical
        draw_vline(stdscr, 2, CMD_BAR_Y, col_div, BOX_V, dim_attr)

        # ============================================================
        # STANGA: INPUT STATIONS
        # ============================================================
        row = 3
        safe_addstr(stdscr, row, col1_x,
                    ' INPUT STATIONS ', bold_attr | curses.A_UNDERLINE)
        row += 1

        inputs = station_data.get('inputs', {})
        for sid, sdata in sorted(inputs.items()):
            if row >= CONTENT_H - 2:
                break

            status  = sdata.get('status', 'IDLE')
            mat     = sdata.get('material_type', '')
            fill    = sdata.get('fill_percent', 0)
            kg      = sdata.get('current_kg', 0)
            items   = sdata.get('current_items', 0)
            max_kg  = sdata.get('max_kg', 0)
            reason  = sdata.get('stop_reason', '')
            metrics = sdata.get('metrics', {})
            pickups      = metrics.get('total_pickups', 0)
            kg_out       = metrics.get('total_weight_transported_kg', 0)
            prod_pct     = metrics.get('producing_percent', 0)
            items_out    = metrics.get('total_items_transported', 0)

            # Linia 1: id + status + material
            safe_addstr(stdscr, row, col1_x, sid, bold_attr)
            safe_addstr(stdscr, row, col1_x + 10, '[')
            colored_status(stdscr, row, col1_x + 11, status)
            safe_addstr(stdscr, row, col1_x + 11 + len(status), ']')
            safe_addstr(stdscr, row, col1_x + 14 + len(status),
                        mat, dim_attr)
            if reason:
                safe_addstr(stdscr, row,
                            col1_x + 14 + len(status) + len(mat) + 1,
                            f'({reason})',
                            curses.color_pair(COLOR_RED))
            row += 1

            # Linia 2: bara progres
            if row < CONTENT_H:
                bar_w = min(22, col1_w - 10)
                draw_progress_bar(stdscr, row, col1_x + 1, bar_w, fill)
                safe_addstr(stdscr, row, col1_x + bar_w + 2,
                            f'{fill:5.1f}%  {kg:.2f}/{max_kg:.1f}kg  {items}pcs',
                            dim_attr)
                row += 1

            # Linia 3: metrici
            if row < CONTENT_H:
                if pickups > 0 or kg_out > 0:
                    safe_addstr(stdscr, row, col1_x + 1,
                                f'pkp:{pickups}  '
                                f'out:{kg_out:.2f}kg({items_out}pcs)  '
                                f'prod:{prod_pct:.0f}%',
                                dim_attr)
                else:
                    safe_addstr(stdscr, row, col1_x + 1,
                                f'prod:{prod_pct:.0f}%  no pickups yet',
                                dim_attr)
                row += 1

            row += 1  # spatiu

        # ---- Separator intre input si output pe stanga ----
        sep_row = row
        if sep_row < CONTENT_H - 4:
            draw_hline(stdscr, sep_row, 0, col_div, BOX_H, dim_attr)
            row += 1

            safe_addstr(stdscr, row, col1_x,
                        ' OUTPUT STATIONS ', bold_attr | curses.A_UNDERLINE)
            row += 1

            outputs = station_data.get('outputs', {})
            for sid, sdata in sorted(outputs.items()):
                if row >= CONTENT_H - 1:
                    break

                status      = sdata.get('status', 'FREE')
                kg          = sdata.get('current_kg', 0)
                reason      = sdata.get('stop_reason', '')
                mats        = ', '.join(sdata.get('accepted_materials', []))
                metrics     = sdata.get('metrics', {})
                deliveries  = metrics.get('total_deliveries', 0)
                kg_in       = metrics.get('total_weight_received_kg', 0)
                util_pct    = metrics.get('utilization_percent', 0)

                # Linia 1: id + status
                safe_addstr(stdscr, row, col1_x, sid, bold_attr)
                safe_addstr(stdscr, row, col1_x + 10, '[')
                colored_status(stdscr, row, col1_x + 11, status)
                safe_addstr(stdscr, row, col1_x + 11 + len(status), ']')
                if kg > 0:
                    safe_addstr(stdscr, row,
                                col1_x + 13 + len(status),
                                f'{kg:.2f}kg')
                if reason:
                    safe_addstr(stdscr, row,
                                col1_x + 22 + len(status),
                                f'({reason})',
                                curses.color_pair(COLOR_RED))
                row += 1

                # Linia 2: metrici
                if row < CONTENT_H:
                    info = f'[{mats}]'
                    if deliveries > 0:
                        info += (
                            f'  del:{deliveries}  '
                            f'in:{kg_in:.2f}kg  '
                            f'util:{util_pct:.0f}%'
                        )
                    safe_addstr(stdscr, row, col1_x + 1, info, dim_attr)
                    row += 1

                row += 1  # spatiu

        # ============================================================
        # DREAPTA: ROBOT + QUEUE + METRICS
        # ============================================================
        rrow = 3

        # ROBOT
        safe_addstr(stdscr, rrow, col2_x,
                    ' ROBOT ', bold_attr | curses.A_UNDERLINE)
        rrow += 1

        safe_addstr(stdscr, rrow, col2_x, 'Status:  ', bold_attr)
        colored_status(stdscr, rrow, col2_x + 9, robot_state)
        rrow += 1

        # Pozitie estimata + AMCL
        est_pos = disp_data.get('robot_pos', {})
        safe_addstr(stdscr, rrow, col2_x,
                    f'Est pos: ({est_pos.get("x", 0):.2f},'
                    f'{est_pos.get("y", 0):.2f})',
                    dim_attr)
        rrow += 1

        if amcl and amcl.get('x') is not None:
            safe_addstr(stdscr, rrow, col2_x,
                        f'AMCL:    ({amcl["x"]:.2f},'
                        f'{amcl["y"]:.2f})',
                        dim_attr)
            rrow += 1

        # Misiune curenta
        current = disp_data.get('current_mission')
        safe_addstr(stdscr, rrow, col2_x, 'Mission: ', bold_attr)
        if current:
            m_input  = current.get('input', '')
            m_output = current.get('output', '')
            m_type   = current.get('type', 'transport')
            elapsed  = current.get('elapsed_sec', 0)
            retries  = current.get('retries', 0)

            if m_type == 'transport' and m_input and m_output:
                mstr = f'{m_input} -> {m_output}'
                safe_addstr(stdscr, rrow, col2_x + 9, mstr,
                            curses.color_pair(COLOR_CYAN) | curses.A_BOLD)
                extra = f'  {elapsed:.0f}s'
                if retries > 0:
                    extra += f'  R:{retries}'
                    safe_addstr(stdscr, rrow,
                                col2_x + 9 + len(mstr),
                                extra,
                                curses.color_pair(COLOR_YELLOW))
                else:
                    safe_addstr(stdscr, rrow,
                                col2_x + 9 + len(mstr),
                                extra, dim_attr)
            elif m_type == 'go_home':
                safe_addstr(stdscr, rrow, col2_x + 9,
                            f'-> HOME  {elapsed:.0f}s',
                            curses.color_pair(COLOR_YELLOW))
            elif m_type == 'undock_home':
                safe_addstr(stdscr, rrow, col2_x + 9,
                            'UNDOCK HOME',
                            curses.color_pair(COLOR_YELLOW))
            else:
                safe_addstr(stdscr, rrow, col2_x + 9,
                            current.get('mission_id', '?'), dim_attr)
        else:
            safe_addstr(stdscr, rrow, col2_x + 9, 'idle', dim_attr)
        rrow += 1

        # Failures
        consec = disp_data.get('consecutive_failures', 0)
        safe_addstr(stdscr, rrow, col2_x, 'Fails:   ', bold_attr)
        if consec > 0:
            safe_addstr(stdscr, rrow, col2_x + 9,
                        str(consec),
                        curses.color_pair(COLOR_RED) | curses.A_BOLD)
        else:
            safe_addstr(stdscr, rrow, col2_x + 9, '0',
                        curses.color_pair(COLOR_GREEN))
        rrow += 2

        # ---- QUEUE ----
        draw_hline(stdscr, rrow, col_div, w, BOX_H, dim_attr)
        rrow += 1
        queue      = disp_data.get('queue', [])
        queue_size = disp_data.get('queue_size', 0)
        safe_addstr(stdscr, rrow, col2_x,
                    f' QUEUE ({queue_size}) ',
                    bold_attr | curses.A_UNDERLINE)
        rrow += 1

        if queue_size == 0:
            safe_addstr(stdscr, rrow, col2_x + 1, 'empty', dim_attr)
            rrow += 1
        else:
            for qi, qm in enumerate(queue):
                if rrow >= CMD_BAR_Y - 6:
                    remaining = queue_size - qi
                    safe_addstr(stdscr, rrow, col2_x + 1,
                                f'... +{remaining} more', dim_attr)
                    rrow += 1
                    break

                q_in     = qm.get('input', '')
                q_out    = qm.get('output', '')
                q_prio   = qm.get('priority', 1)
                q_ret    = qm.get('retries', 0)
                q_age    = qm.get('queue_age_sec', 0)
                q_weight = qm.get('weight_kg', 0)

                line = f' {qi+1}. {q_in}->{q_out}'
                line += f'  p={q_prio}'
                if q_weight > 0:
                    line += f'  {q_weight:.2f}kg'
                if q_age > 5:
                    line += f'  age:{q_age:.0f}s'
                if q_ret > 0:
                    line += f'  R:{q_ret}'

                attr = (curses.color_pair(COLOR_CYAN) | curses.A_BOLD
                        if q_prio == 0 else 0)
                safe_addstr(stdscr, rrow, col2_x, line, attr)
                rrow += 1

        rrow += 1

        # ---- METRICS ----
        if rrow < CMD_BAR_Y - 3:
            draw_hline(stdscr, rrow, col_div, w, BOX_H, dim_attr)
            rrow += 1
            safe_addstr(stdscr, rrow, col2_x,
                        ' METRICS ', bold_attr | curses.A_UNDERLINE)
            rrow += 1

            total_m  = disp_data.get('total_missions', 0)
            ok_m     = disp_data.get('successful', 0)
            fail_m   = disp_data.get('failed', 0)
            util_pct = disp_data.get('robot_utilization_pct', 0)
            up_sec   = disp_data.get('uptime_sec', 0)

            rate    = (total_m / (up_sec / 3600)) if up_sec > 60 else 0
            ok_pct  = (ok_m / total_m * 100) if total_m > 0 else 0

            if rrow < CMD_BAR_Y - 1:
                # Missions line
                safe_addstr(stdscr, rrow, col2_x, 'Missions:', bold_attr)
                safe_addstr(stdscr, rrow, col2_x + 10,
                            str(ok_m),
                            curses.color_pair(COLOR_GREEN) | curses.A_BOLD)
                safe_addstr(stdscr, rrow,
                            col2_x + 10 + len(str(ok_m)),
                            'ok  ', dim_attr)
                safe_addstr(stdscr, rrow,
                            col2_x + 14 + len(str(ok_m)),
                            str(fail_m),
                            curses.color_pair(COLOR_RED) | curses.A_BOLD
                            if fail_m > 0
                            else dim_attr)
                safe_addstr(stdscr, rrow,
                            col2_x + 14 + len(str(ok_m)) + len(str(fail_m)),
                            f'fail  {total_m}total', dim_attr)
                rrow += 1

            if rrow < CMD_BAR_Y - 1:
                safe_addstr(stdscr, rrow, col2_x,
                            f'Rate:{rate:5.1f}/hr  '
                            f'OK:{ok_pct:.0f}%  '
                            f'Util:{util_pct:.0f}%',
                            dim_attr)
                rrow += 1

        # ============================================================
        # CMD BAR
        # ============================================================
        draw_hline(stdscr, CMD_BAR_Y, 0, w, BOX_H, dim_attr)

        # Rand 1 de comenzi: keybinds
        cx = 1
        for kb_key, kb_label, _ in KEYBINDS:
            chunk = f'[{kb_key}]{kb_label} '
            if cx + len(chunk) >= w - 2:
                break
            safe_addstr(stdscr, CMD_BAR_Y + 1, cx,
                        f'[{kb_key}]', bold_attr)
            safe_addstr(stdscr, CMD_BAR_Y + 1,
                        cx + len(f'[{kb_key}]'),
                        kb_label + ' ', dim_attr)
            cx += len(chunk)

        # Feedback comanda trimisa (2 secunde)
        feedback = node.get_cmd_feedback()
        if feedback:
            fb_attr = curses.color_pair(COLOR_GREEN) | curses.A_BOLD
            safe_addstr(stdscr, CMD_BAR_Y + 1,
                        w - len(feedback) - 2,
                        feedback, fb_attr)

        # ============================================================
        # FOOTER
        # ============================================================
        total_m = disp_data.get('total_missions', 0)
        ok_m    = disp_data.get('successful', 0)
        fail_m  = disp_data.get('failed', 0)

        footer = (
            f' {ok_m}/{total_m} missions'
            f'  {fail_m} failed'
            f'  | [Q]uit'
            f'  | NOUZEN v2 '
        )
        safe_addstr(stdscr, FOOTER_Y, 0,
                    footer.ljust(w - 1), header_attr)

        stdscr.refresh()


def main():
    rclpy.init()
    node = DashboardNode()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        curses.wrapper(lambda stdscr: draw_dashboard(stdscr, node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()