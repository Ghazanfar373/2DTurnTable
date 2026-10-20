"""
Attitude and servo window for the Mega sketch.

  py -3 -m pip install -r gui/requirements.txt
  py -3 gui/imu_servo_gui.py

Serial: 115200. Attitude frames are 15 bytes starting with AA 02.
Servo commands are 6 bytes starting with AA 01 (id, pulse in microseconds).
"""

import math
import struct
import threading
import time
import tkinter as tk
from tkinter import ttk

SYNC = 0xAA
CMD_SET_US = 0x01
CMD_ATTITUDE = 0x02
CMD_ACK = 0x81
ATT_LEN = 15
SERVO_LEN = 6

SERVO_MIN_US = 1000
SERVO_MAX_US = 2000
SERVO_CENTER_US = 1500
# TD8135MG: 500–2500 µs is 180° at the horn, so this pulse span is ±45°.
US_PER_DEG = 2000.0 / 180.0
HORN_MIN_DEG = (SERVO_MIN_US - SERVO_CENTER_US) / US_PER_DEG
HORN_MAX_DEG = (SERVO_MAX_US - SERVO_CENTER_US) / US_PER_DEG
# Measured pulse widths for the two pitch positions: (servo 1, servo 2).
PITCH_UP_US = (1620, 1850)
PITCH_DOWN_US = (1040, 1000)
# (servo 1 µs, servo 2 µs). Measured the same way as the pitch positions.
ROLL_LEFT_US = (1000, 1850)
ROLL_RIGHT_US = (1720, 1000)
BAUD = 115200

BG = "#101418"
PANEL = "#1a212b"
CANVAS_BG = "#0c1218"
TEXT = "#e7edf4"
MUTED = "#8b98a8"
ACCENT = "#3d8bfd"
GRID = "#243140"
SPHERE = "#2c3d52"
SKY = "#1d6ec2"
GROUND = "#7a4e2d"
HUD_LINE = "#f4f7fb"
HUD_MARK = "#f5d76e"


def xor_bytes(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


def us_to_deg(pulse_us):
    return (float(pulse_us) - SERVO_CENTER_US) / US_PER_DEG


def deg_to_us(deg):
    pulse = SERVO_CENTER_US + float(deg) * US_PER_DEG
    return max(SERVO_MIN_US, min(SERVO_MAX_US, pulse))


def servo_packet(servo_id, pulse_us):
    pulse = max(SERVO_MIN_US, min(SERVO_MAX_US, int(pulse_us)))
    body = bytes((SYNC, CMD_SET_US, servo_id, pulse & 0xFF, (pulse >> 8) & 0xFF))
    return body + bytes((xor_bytes(body),))


def rot_x(point, angle):
    x, y, z = point
    c, s = math.cos(angle), math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def rot_y(point, angle):
    x, y, z = point
    c, s = math.cos(angle), math.sin(angle)
    return (x * c + z * s, y, -x * s + z * c)


def rot_z(point, angle):
    x, y, z = point
    c, s = math.cos(angle), math.sin(angle)
    return (x * c - y * s, x * s + y * c, z)


def body_to_world(point, pitch_deg, roll_deg, yaw_deg):
    # Sensor signs: +pitch nose up, +roll right wing down, +yaw nose right.
    point = rot_x(point, -math.radians(roll_deg))
    point = rot_y(point, -math.radians(pitch_deg))
    point = rot_z(point, math.radians(yaw_deg))
    return point


def world_to_view(point):
    x, y, z = point
    point = (y, z, x)
    point = rot_y(point, math.radians(-32))
    point = rot_x(point, math.radians(22))
    return point


def delta_uav_faces():
    """Flying-wing delta. +X is the nose, +Y is right, +Z is up."""
    faces = []

    def tri(color, a, b, c):
        faces.append((color, (a, b, c)))

    def quad(color, a, b, c, d):
        faces.append((color, (a, b, c, d)))

    nose = (1.70, 0.0)
    tip_r = (-1.20, 2.20)
    tip_l = (-1.20, -2.20)
    top, bot = 0.06, -0.05

    def at(point, height):
        return (point[0], point[1], height)

    n_t, r_t, l_t = at(nose, top), at(tip_r, top), at(tip_l, top)
    n_b, r_b, l_b = at(nose, bot), at(tip_r, bot), at(tip_l, bot)
    tri("#5eb0ff", n_t, r_t, l_t)
    tri("#1d4e89", n_b, l_b, r_b)
    quad("#8ec8ff", n_t, r_t, r_b, n_b)
    quad("#8ec8ff", n_t, l_b, l_t, n_b)
    quad("#163e70", l_t, r_t, r_b, l_b)

    # Center pod so the nose stays obvious when the wing is edge-on.
    pod_n = (1.25, 0.0, 0.28)
    pod_r = (0.05, 0.32, 0.08)
    pod_l = (0.05, -0.32, 0.08)
    pod_t = (-0.95, 0.0, 0.10)
    tri("#e74c3c", pod_n, pod_r, pod_l)
    tri("#d5dbe3", pod_n, pod_t, pod_r)
    tri("#c5ced8", pod_n, pod_l, pod_t)
    tri("#9aa3b2", pod_t, pod_l, pod_r)

    def fin(y):
        base_f = (-0.45, y, top)
        base_a = (-1.05, y, top)
        peak = (-0.82, y, 0.62)
        shift = 0.05 if y > 0 else -0.05
        tri("#c0392b", base_f, base_a, peak)
        tri("#922b21",
            (base_f[0], base_f[1] + shift, base_f[2]),
            (base_a[0], base_a[1] + shift, base_a[2]),
            (peak[0], peak[1] + shift, peak[2]))

    fin(1.35)
    fin(-1.35)
    return faces


def sphere_lines(radius=3.15, meridians=10, parallels=6):
    lines = []
    for index in range(meridians):
        lon = 2 * math.pi * index / meridians
        ring = []
        for step in range(25):
            lat = -math.pi / 2 + math.pi * step / 24
            ring.append((
                radius * math.cos(lat) * math.cos(lon),
                radius * math.cos(lat) * math.sin(lon),
                radius * math.sin(lat),
            ))
        lines.append(ring)
    for index in range(1, parallels):
        lat = -math.pi / 2 + math.pi * index / parallels
        ring = []
        for step in range(33):
            lon = 2 * math.pi * step / 32
            ring.append((
                radius * math.cos(lat) * math.cos(lon),
                radius * math.cos(lat) * math.sin(lon),
                radius * math.sin(lat),
            ))
        lines.append(ring)
    return lines


def ground_lines(size=3.0, step=1.0):
    lines = []
    value = -size
    while value <= size + 1e-6:
        lines.append(((-size, value, 0.0), (size, value, 0.0)))
        lines.append(((value, -size, 0.0), (value, size, 0.0)))
        value += step
    return lines


class PacketParser:
    def __init__(self):
        self.buf = bytearray()
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.have_attitude = False
        self.last_attitude = 0.0
        self.last_ack = ""

    def feed(self, data):
        self.buf.extend(data)
        while self.buf:
            if self.buf[0] != SYNC:
                del self.buf[0]
                continue
            if len(self.buf) < 2:
                return
            kind = self.buf[1]
            if kind == CMD_ATTITUDE:
                need = ATT_LEN
            elif kind in (CMD_SET_US, CMD_ACK):
                need = SERVO_LEN
            else:
                del self.buf[0]
                continue
            if len(self.buf) < need:
                return
            frame = bytes(self.buf[:need])
            if xor_bytes(frame[:-1]) != frame[-1]:
                del self.buf[0]
                continue
            del self.buf[:need]
            self._accept(frame)

    def _accept(self, frame):
        if frame[1] == CMD_ATTITUDE:
            self.pitch, self.roll, self.yaw = struct.unpack_from("<fff", frame, 2)
            self.have_attitude = True
            self.last_attitude = time.monotonic()
        elif frame[1] == CMD_ACK:
            pulse = frame[3] | (frame[4] << 8)
            self.last_ack = "Servo %d at %d us" % (frame[2], pulse)


class AttitudeView(tk.Canvas):
    def __init__(self, master):
        super().__init__(
            master, bg=CANVAS_BG, highlightthickness=0, width=480, height=680,
        )
        self.faces = delta_uav_faces()
        self.sphere = sphere_lines()
        self.ground = ground_lines()
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.bind("<Configure>", self._on_resize)
        self.view_w = 480
        self.view_h = 680

    def _on_resize(self, event):
        self.view_w = max(event.width, 2)
        self.view_h = max(event.height, 2)

    def set_attitude(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self._draw()

    def _project(self, point, rotate):
        if rotate:
            point = body_to_world(point, self.pitch, self.roll, self.yaw)
        x, y, z = world_to_view(point)
        denom = 8.2 - z
        if denom < 0.35:
            denom = 0.35
        # 0.90 puts the sphere diameter on the HUD circle (radius 0.36 of the short side).
        scale = min(self.view_w, self.view_h) * 0.90 / denom
        return (
            self.view_w * 0.5 + x * scale,
            self.view_h * 0.46 - y * scale,
            denom,
        )

    def _draw(self):
        self.delete("all")
        width, height = self.view_w, self.view_h
        self.create_text(
            18, 16, anchor="nw", fill=MUTED,
            font=("Segoe UI", 11), text="Fixed sphere, delta UAV follows the IMU",
        )

        for line in self.ground:
            ax, ay, _ = self._project(line[0], False)
            bx, by, _ = self._project(line[1], False)
            self.create_line(ax, ay, bx, by, fill=GRID)

        for ring in self.sphere:
            flat = []
            for point in ring:
                sx, sy, _ = self._project(point, False)
                flat.extend((sx, sy))
            self.create_line(*flat, fill=SPHERE, smooth=False)

        painted = []
        for color, corners in self.faces:
            projected = [self._project(corner, True) for corner in corners]
            depth = sum(item[2] for item in projected) / len(projected)
            painted.append((depth, color, projected))
        painted.sort(key=lambda item: item[0], reverse=True)
        for _depth, color, projected in painted:
            coords = []
            for sx, sy, _ in projected:
                coords.extend((sx, sy))
            self.create_polygon(*coords, fill=color, outline="#071018")

        axis = (
            ((1.5, 0, 0), "#e74c3c", "X"),
            ((0, 1.5, 0), "#2ecc71", "Y"),
            ((0, 0, 1.5), "#3498db", "Z"),
        )
        origin = self._project((0, 0, 0), True)
        for tip, color, label in axis:
            end = self._project(tip, True)
            self.create_line(origin[0], origin[1], end[0], end[1], fill=color, width=2)
            self.create_text(end[0], end[1], text=label, fill=color, font=("Segoe UI", 10, "bold"))

        self.create_text(
            18, height - 18, anchor="sw", fill=TEXT, font=("Segoe UI", 12),
            text="P %6.1f°    R %6.1f°    Y %6.1f°" % (self.pitch, self.roll, self.yaw),
        )


class HorizonView(tk.Canvas):
    """Pilot view: the horizon stays level in the world, so it rolls and pitches
    opposite the airplane. The yellow wings stay fixed in the center."""

    def __init__(self, master):
        super().__init__(master, bg=CANVAS_BG, highlightthickness=0, width=480, height=680)
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.view_w = 480
        self.view_h = 680
        self.bind("<Configure>", self._on_resize)

    def _on_resize(self, event):
        self.view_w = max(event.width, 2)
        self.view_h = max(event.height, 2)

    def set_attitude(self, pitch, roll, yaw):
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self._draw()

    def _screen(self, x, y, cx, cy):
        return cx + x, cy - y

    def _rotate(self, x, y):
        angle = math.radians(self.roll)
        c, s = math.cos(angle), math.sin(angle)
        return (x * c - y * s, x * s + y * c)

    def _arc(self, radius, start, sweep, steps):
        count = max(int(abs(sweep) / (2 * math.pi) * steps), 2)
        return [
            (radius * math.cos(start + sweep * i / count),
             radius * math.sin(start + sweep * i / count))
            for i in range(count + 1)
        ]

    def _region(self, radius, normal, dist, toward_sky):
        if dist <= -radius:
            return "all" if toward_sky else "none"
        if dist >= radius:
            return "none" if toward_sky else "all"
        half = math.sqrt(max(radius * radius - dist * dist, 0.0))
        tx, ty = normal[1], -normal[0]
        p1 = (normal[0] * dist + tx * half, normal[1] * dist + ty * half)
        p2 = (normal[0] * dist - tx * half, normal[1] * dist - ty * half)
        a1 = math.atan2(p1[1], p1[0])
        a2 = math.atan2(p2[1], p2[0])
        ccw = (a2 - a1) % (2 * math.pi)
        mid = a1 + ccw * 0.5
        sky_mid = math.cos(mid) * normal[0] + math.sin(mid) * normal[1] >= 0
        if sky_mid == toward_sky:
            sweep = ccw
        else:
            sweep = ccw - 2 * math.pi
        return self._arc(radius, a1, sweep, 64)

    def _draw(self):
        self.delete("all")
        width, height = self.view_w, self.view_h
        cx, cy = width * 0.5, height * 0.46
        radius = min(width, height) * 0.36
        px_per_deg = radius / 28.0
        self.create_text(
            18, 16, anchor="nw", fill=MUTED, font=("Segoe UI", 11),
            text="Horizon, pilot view",
        )

        theta = math.radians(self.roll)
        normal = (-math.sin(theta), math.cos(theta))
        dist = -self.pitch * px_per_deg
        box = (cx - radius, cy - radius, cx + radius, cy + radius)

        sky = self._region(radius, normal, dist, True)
        ground = self._region(radius, normal, dist, False)
        if sky == "all":
            self.create_oval(*box, fill=SKY, outline="")
        elif sky != "none":
            coords = []
            for x, y in sky:
                sx, sy = self._screen(x, y, cx, cy)
                coords.extend((sx, sy))
            self.create_polygon(*coords, fill=SKY, outline="")
        if ground == "all":
            self.create_oval(*box, fill=GROUND, outline="")
        elif ground != "none":
            coords = []
            for x, y in ground:
                sx, sy = self._screen(x, y, cx, cy)
                coords.extend((sx, sy))
            self.create_polygon(*coords, fill=GROUND, outline="")

        half = math.sqrt(max(radius * radius - dist * dist, 0.0)) if abs(dist) < radius else 0.0
        if half > 0:
            tx, ty = normal[1], -normal[0]
            h1 = self._screen(normal[0] * dist + tx * half, normal[1] * dist + ty * half, cx, cy)
            h2 = self._screen(normal[0] * dist - tx * half, normal[1] * dist - ty * half, cx, cy)
            self.create_line(*h1, *h2, fill=HUD_LINE, width=3)

        for mark in range(-60, 70, 10):
            if mark == 0:
                continue
            y = -(self.pitch - mark) * px_per_deg
            mark_w = 42 if mark % 30 == 0 else 22
            x1, y1 = self._rotate(-mark_w, y)
            x2, y2 = self._rotate(mark_w, y)
            if x1 * x1 + y1 * y1 > (radius - 10) ** 2 or x2 * x2 + y2 * y2 > (radius - 10) ** 2:
                continue
            s1 = self._screen(x1, y1, cx, cy)
            s2 = self._screen(x2, y2, cx, cy)
            self.create_line(*s1, *s2, fill=HUD_LINE, width=2)
            lx, ly = self._rotate(-mark_w - 16, y)
            if lx * lx + ly * ly < (radius - 14) ** 2:
                sl = self._screen(lx, ly, cx, cy)
                self.create_text(sl[0], sl[1], text=str(abs(mark)), fill=HUD_LINE, font=("Segoe UI", 8))

        for deg in (0, 10, 20, 30, 45, 60):
            signs = (0,) if deg == 0 else (-1, 1)
            for sign in signs:
                angle = math.radians(90 - sign * deg)
                inner = radius - (20 if deg % 30 == 0 or deg == 0 else 12)
                outer = radius - 2
                a = (inner * math.cos(angle), inner * math.sin(angle))
                b = (outer * math.cos(angle), outer * math.sin(angle))
                self.create_line(*self._screen(*a, cx, cy), *self._screen(*b, cx, cy), fill=HUD_LINE, width=2)

        pointer = math.radians(90 - self.roll)
        tip_r = radius - 28
        tip = (tip_r * math.cos(pointer), tip_r * math.sin(pointer))
        base = pointer + math.pi
        left = (tip[0] + 9 * math.cos(base + 0.5), tip[1] + 9 * math.sin(base + 0.5))
        right = (tip[0] + 9 * math.cos(base - 0.5), tip[1] + 9 * math.sin(base - 0.5))
        tri = []
        for point in (tip, left, right):
            sx, sy = self._screen(*point, cx, cy)
            tri.extend((sx, sy))
        self.create_polygon(*tri, fill=HUD_MARK, outline="")

        self.create_oval(*box, outline="#071018", width=14)
        self.create_oval(*box, outline="#314155", width=2)

        gap, wing = 16, radius * 0.28
        self.create_line(cx - gap - wing, cy, cx - gap, cy, fill=HUD_MARK, width=4, capstyle="round")
        self.create_line(cx + gap, cy, cx + gap + wing, cy, fill=HUD_MARK, width=4, capstyle="round")
        self.create_line(cx - gap, cy, cx - gap - 10, cy + 10, fill=HUD_MARK, width=4, capstyle="round")
        self.create_line(cx + gap, cy, cx + gap + 10, cy + 10, fill=HUD_MARK, width=4, capstyle="round")
        self.create_oval(cx - 4, cy - 4, cx + 4, cy + 4, fill=HUD_MARK, outline=HUD_MARK)

        heading = self.yaw % 360.0
        self.create_text(
            cx, cy + radius + 28, fill=TEXT, font=("Segoe UI", 11),
            text="Pitch %+0.1f°    Roll %+0.1f°    Hdg %0.0f°" % (self.pitch, self.roll, heading),
        )


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("IMU attitude and servos")
        self.configure(bg=BG)
        self.geometry("1280x720")
        self.minsize(1080, 600)
        self.parser = PacketParser()
        self.serial = None
        self.reader = None
        self.running = False
        self.io_lock = threading.Lock()
        self._step_job = None
        self._step_t = 0.0
        self._ignore_slider = False
        self.goal_us = [SERVO_CENTER_US, SERVO_CENTER_US]
        self.out_us = [float(SERVO_CENTER_US), float(SERVO_CENTER_US)]
        self._last_sent = [None, None]
        self._ui_ready = False

        stage = tk.Frame(self, bg=BG)
        stage.pack(side="left", fill="both", expand=True)
        stage.columnconfigure(0, weight=1, uniform="views")
        stage.columnconfigure(1, weight=1, uniform="views")
        stage.rowconfigure(0, weight=1)
        self.view = AttitudeView(stage)
        self.horizon = HorizonView(stage)
        self.view.grid(row=0, column=0, sticky="nsew")
        self.horizon.grid(row=0, column=1, sticky="nsew")

        panel = tk.Frame(self, bg=PANEL, width=300)
        panel.pack(side="right", fill="y")
        panel.pack_propagate(False)

        self._section(panel, "Serial")
        row = tk.Frame(panel, bg=PANEL)
        row.pack(fill="x", padx=16, pady=(0, 8))
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(row, textvariable=self.port_var, state="readonly", width=16)
        self.port_box.pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Refresh", command=self.refresh_ports).pack(side="left", padx=(8, 0))

        self.connect_btn = ttk.Button(panel, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(fill="x", padx=16, pady=(0, 6))
        tk.Label(panel, text="115200 baud", bg=PANEL, fg=MUTED, font=("Segoe UI", 9)).pack(anchor="w", padx=16)

        self._section(panel, "Attitude")
        self.pitch_var = tk.StringVar(value="0.0°")
        self.roll_var = tk.StringVar(value="0.0°")
        self.yaw_var = tk.StringVar(value="0.0°")
        self._readout(panel, "Pitch", self.pitch_var)
        self._readout(panel, "Roll", self.roll_var)
        self._readout(panel, "Yaw", self.yaw_var)

        self._section(panel, "Servos")
        tk.Label(
            panel, text="Horn angle, −45° to +45°",
            bg=PANEL, fg=MUTED, font=("Segoe UI", 9),
        ).pack(anchor="w", padx=16, pady=(0, 8))
        self.servo_vars = {}
        self.servo_scales = {}
        self.servo_labels = {}
        for servo_id in (1, 2):
            self._slider(panel, servo_id)
        tk.Label(
            panel, text="Move time", bg=PANEL, fg=TEXT, font=("Segoe UI", 11),
        ).pack(anchor="w", padx=16, pady=(14, 0))
        self.move_time_var = tk.IntVar(value=8)
        self.move_time_label = tk.StringVar(value="8 s")
        tk.Label(
            panel, textvariable=self.move_time_label, bg=PANEL, fg=ACCENT, font=("Segoe UI", 10),
        ).pack(anchor="w", padx=16)
        tk.Scale(
            panel, from_=2, to=30, orient="horizontal", resolution=1, showvalue=False,
            variable=self.move_time_var, bg=ACCENT, troughcolor="#0e141b",
            activebackground="#6cb2ff", highlightthickness=0, bd=0,
            sliderrelief="flat", sliderlength=22, width=14,
            command=lambda value: self.move_time_label.set("%s s" % int(float(value))),
        ).pack(fill="x", padx=16)
        tk.Label(
            panel, text="Full horn travel uses this time",
            bg=PANEL, fg=MUTED, font=("Segoe UI", 9),
        ).pack(anchor="w", padx=16, pady=(0, 4))
        ttk.Button(panel, text="Center both", command=self.center_servos).pack(fill="x", padx=16, pady=(12, 0))
        ttk.Button(panel, text="Pitch up 5°", command=lambda: self.move_servos_slow(PITCH_UP_US)).pack(fill="x", padx=16, pady=(8, 0))
        ttk.Button(panel, text="Pitch Down 5°", command=lambda: self.move_servos_slow(PITCH_DOWN_US)).pack(fill="x", padx=16, pady=(8, 0))
        ttk.Button(panel, text="Roll left 5°", command=lambda: self.move_servos_slow(ROLL_LEFT_US)).pack(fill="x", padx=16, pady=(8, 0))
        ttk.Button(panel, text="Roll Right 5°", command=lambda: self.move_servos_slow(ROLL_RIGHT_US)).pack(fill="x", padx=16, pady=(8, 0))

        self.status_var = tk.StringVar(value="Disconnected")
        tk.Label(
            panel, textvariable=self.status_var, bg=PANEL, fg=ACCENT,
            font=("Segoe UI", 10), wraplength=260, justify="left",
        ).pack(anchor="w", padx=16, pady=18)

        self.refresh_ports()
        self._ui_ready = True
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(33, self.tick)
        self.view.set_attitude(0, 0, 0)
        self.horizon.set_attitude(0, 0, 0)

    def _section(self, parent, title):
        tk.Label(
            parent, text=title, bg=PANEL, fg=TEXT,
            font=("Segoe UI", 13, "bold"),
        ).pack(anchor="w", padx=16, pady=(18, 8))

    def _readout(self, parent, name, variable):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", padx=16, pady=2)
        tk.Label(row, text=name, bg=PANEL, fg=MUTED, font=("Segoe UI", 11), width=8, anchor="w").pack(side="left")
        tk.Label(row, textvariable=variable, bg=PANEL, fg=TEXT, font=("Segoe UI", 16)).pack(side="left")

    def _slider(self, parent, servo_id):
        tk.Label(
            parent, text="Servo %d" % servo_id, bg=PANEL, fg=TEXT, font=("Segoe UI", 11),
        ).pack(anchor="w", padx=16, pady=(8, 0))
        variable = tk.DoubleVar(value=0.0)
        self.servo_vars[servo_id] = variable
        value = tk.StringVar(value="+0.0°    1500 µs")
        self.servo_labels[servo_id] = value
        tk.Label(parent, textvariable=value, bg=PANEL, fg=ACCENT, font=("Segoe UI", 10)).pack(anchor="w", padx=16)
        scale = tk.Scale(
            parent, from_=HORN_MIN_DEG, to=HORN_MAX_DEG, orient="horizontal",
            resolution=0.1, showvalue=False, variable=variable,
            bg=ACCENT, fg=TEXT, troughcolor="#0e141b",
            activebackground="#6cb2ff", highlightthickness=0, bd=0,
            sliderrelief="flat", sliderlength=22, width=14,
            command=lambda value, sid=servo_id: self.on_slider(sid, value),
        )
        scale.pack(fill="x", padx=16)
        self.servo_scales[servo_id] = scale

    def refresh_ports(self):
        ports = []
        try:
            from serial.tools import list_ports
            ports = [item.device for item in list_ports.comports()]
        except Exception:
            ports = []
        self.port_box["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def toggle_connection(self):
        if self.running:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("No serial port selected")
            return
        try:
            import serial
        except ImportError:
            self.status_var.set("Install pyserial: py -3 -m pip install pyserial")
            return
        try:
            self.serial = serial.Serial(port, BAUD, timeout=0.05)
        except Exception as exc:
            self.serial = None
            self.status_var.set(str(exc))
            return
        self.parser = PacketParser()
        self.running = True
        self.reader = threading.Thread(target=self._read_loop, daemon=True)
        self.reader.start()
        self.connect_btn.configure(text="Disconnect")
        self.status_var.set("Connected to %s" % port)
        self.center_servos()

    def disconnect(self):
        self.running = False
        thread = self.reader
        port = self.serial
        self.reader = None
        self.serial = None
        if port is not None:
            try:
                port.close()
            except Exception:
                pass
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.4)
        self.connect_btn.configure(text="Connect")
        self.status_var.set("Disconnected")

    def _read_loop(self):
        while self.running and self.serial is not None:
            try:
                data = self.serial.read(256)
            except Exception:
                self.running = False
                self.after(0, self._on_link_lost)
                return
            if data:
                self.parser.feed(data)

    def _on_link_lost(self):
        if self.serial is not None:
            self.disconnect()
        else:
            self.connect_btn.configure(text="Connect")
            self.status_var.set("Serial link lost")

    def on_slider(self, servo_id, value=None):
        if not self._ui_ready or self._ignore_slider:
            return
        pulse = self._pulse_from_degrees(value if value is not None else self.servo_vars[servo_id].get())
        index = servo_id - 1
        # Button presses set the knob to the goal. A matching callback is not a new drag.
        if pulse == self.goal_us[index]:
            self._refresh_servo_label(servo_id)
            return
        self.goal_us[index] = pulse
        self._refresh_servo_label(servo_id)
        self._ensure_stepper()

    def _pulse_from_degrees(self, degrees):
        return int(round(deg_to_us(float(degrees))))

    def _refresh_servo_label(self, servo_id):
        index = servo_id - 1
        goal = self.goal_us[index]
        out = self.out_us[index]
        if abs(out - goal) > 2:
            text = "%+.1f° -> %+.1f°" % (us_to_deg(out), us_to_deg(goal))
        else:
            text = "%+.1f°    %d µs" % (us_to_deg(goal), int(round(goal)))
        self.servo_labels[servo_id].set(text)

    def _ensure_stepper(self):
        if self._step_job is None:
            self._step_t = time.monotonic()
            self._step_job = self.after(20, self._step_once)

    def _step_once(self):
        self._step_job = None
        try:
            now = time.monotonic()
            dt = min(0.2, max(0.02, now - self._step_t))
            self._step_t = now
            seconds = max(2, int(self.move_time_var.get()))
            # Full −45°..+45° horn travel takes Move time seconds.
            rate = (SERVO_MAX_US - SERVO_MIN_US) / float(seconds)
            advance = rate * dt
            busy = False
            for index, servo_id in enumerate((1, 2)):
                goal = float(self.goal_us[index])
                current = self.out_us[index]
                error = goal - current
                nxt = current + math.copysign(min(abs(error), advance), error)
                # Finish on the exact pulse instead of a last step inside the deadband.
                if abs(goal - nxt) < 8:
                    nxt = goal
                else:
                    busy = True
                self.out_us[index] = nxt
                self._send_step(servo_id, int(round(nxt)), force=(nxt == goal))
                self._refresh_servo_label(servo_id)
            if busy:
                self._step_job = self.after(20, self._step_once)
        except Exception as exc:
            self.status_var.set("Servo move failed: %s" % exc)

    def _send_step(self, servo_id, pulse, force):
        index = servo_id - 1
        previous = self._last_sent[index]
        # TD8135MG ignores pulse changes of about 5 µs, so smaller steps never move.
        if previous is not None and not force and abs(pulse - previous) < 8:
            return
        if pulse == previous:
            return
        self._last_sent[index] = pulse
        self._write_servo(servo_id, pulse)

    def _write_servo(self, servo_id, pulse):
        if not self.running or self.serial is None:
            return
        packet = servo_packet(servo_id, pulse)
        try:
            with self.io_lock:
                self.serial.write(packet)
        except Exception:
            self.after(0, self._on_link_lost)

    def center_servos(self):
        self.move_servos_slow((SERVO_CENTER_US, SERVO_CENTER_US))

    def move_servos_slow(self, pulses):
        if not pulses:
            self.status_var.set("Pulse widths for this move are not set yet")
            return
        self._ignore_slider = True
        try:
            for index, (servo_id, pulse) in enumerate(zip((1, 2), pulses)):
                pulse = max(SERVO_MIN_US, min(SERVO_MAX_US, int(pulse)))
                self.goal_us[index] = pulse
                self.servo_scales[servo_id].set(us_to_deg(pulse))
                self._refresh_servo_label(servo_id)
        finally:
            self._ignore_slider = False
        self._ensure_stepper()

    def _cancel_stepper(self):
        if self._step_job is not None:
            self.after_cancel(self._step_job)
            self._step_job = None

    def tick(self):
        pitch, roll, yaw = self.parser.pitch, self.parser.roll, self.parser.yaw
        self.view.set_attitude(pitch, roll, yaw)
        self.horizon.set_attitude(pitch, roll, yaw)
        self.pitch_var.set("%0.1f°" % pitch)
        self.roll_var.set("%0.1f°" % roll)
        self.yaw_var.set("%0.1f°" % yaw)
        if self.running:
            if self.parser.have_attitude and time.monotonic() - self.parser.last_attitude < 0.5:
                ack = self.parser.last_ack
                self.status_var.set(ack or "Receiving attitude")
            elif self.parser.have_attitude:
                self.status_var.set("No recent attitude packet")
            else:
                self.status_var.set("Waiting for attitude (gyro calibration)")
        self.after(33, self.tick)

    def on_close(self):
        self._cancel_stepper()
        self.disconnect()
        self.destroy()


def main():
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        pass
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
