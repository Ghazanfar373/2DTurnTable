"""
Stabilization test page for stabilize/stabilize.ino.

  py -3 gui/stabilize_gui.py

Does not change the working servo GUI. Upload stabilize/stabilize.ino
instead of mpu_freertos_servo.ino before connecting.

Serial is still 115200. Attitude is the 15-byte AA 02 packet.
Servo trim is the 6-byte AA 01 packet. Stabilizer settings are a
25-byte AA 03 packet.
"""

import struct
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

sys.path.insert(0, str(Path(__file__).resolve().parent))

from imu_servo_gui import (  # noqa: E402
    ACCENT,
    ATT_LEN,
    AttitudeView,
    BAUD,
    BG,
    CMD_ACK,
    CMD_ATTITUDE,
    CMD_SET_US,
    HorizonView,
    MUTED,
    PANEL,
    SERVO_LEN,
    SYNC,
    TEXT,
    servo_packet,
    xor_bytes,
)

CMD_STAB = 0x03
STAB_LEN = 25
# IMU angles measured at each 5° table pose: (target pitch, target roll).
PITCH_UP_DEG = (29.0, 4.0)
PITCH_DOWN_DEG = (-24.5, 4.0)
ROLL_LEFT_DEG = (3.0, -10.5)
ROLL_RIGHT_DEG = (3.0, 14.0)


def i16(value, lo, hi):
    value = int(round(value))
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def stab_packet(enable, kp, ki, kd, mix, target_pitch, target_roll, slew):
    body = bytearray(24)
    body[0] = SYNC
    body[1] = CMD_STAB
    body[2] = 1 if enable else 0
    struct.pack_into(
        "<hhh", body, 4,
        i16(kp * 1000, 0, 5000),
        i16(ki * 1000, 0, 2000),
        i16(kd * 1000, 0, 2000),
    )
    struct.pack_into("<hhhh", body, 10, *[i16(item, -200, 200) for item in mix])
    struct.pack_into(
        "<hhH", body, 18,
        i16(target_pitch * 100, -4500, 4500),
        i16(target_roll * 100, -4500, 4500),
        i16(slew, 20, 800),
    )
    return bytes(body) + bytes((xor_bytes(body),))


class PacketParser:
    def __init__(self):
        self.buf = bytearray()
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.have_attitude = False
        self.last_attitude = 0.0
        self.servo_us = [1500, 1500]
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
            elif kind == CMD_STAB:
                need = STAB_LEN
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
        elif frame[1] == CMD_ACK and frame[2] in (1, 2):
            pulse = frame[3] | (frame[4] << 8)
            self.servo_us[frame[2] - 1] = pulse
            self.last_ack = "Servo %d at %d us" % (frame[2], pulse)


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Stabilization test")
        self.configure(bg=BG)
        self.geometry("1280x760")
        self.minsize(1080, 640)
        self.parser = PacketParser()
        self.serial = None
        self.reader = None
        self.running = False
        self.io_lock = threading.Lock()
        self._cfg_job = None
        self._trim_job = None
        self._pending_trim = set()
        self._ignore_trim = False
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

        panel = tk.Frame(self, bg=PANEL, width=360)
        panel.pack(side="right", fill="y")
        panel.pack_propagate(False)
        self._build_panel(panel)

        self.refresh_ports()
        self._ui_ready = True
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(33, self.tick)
        self.view.set_attitude(0, 0, 0)
        self.horizon.set_attitude(0, 0, 0)

    def _build_panel(self, panel):
        canvas = tk.Canvas(panel, bg=PANEL, highlightthickness=0, bd=0)
        scroll = ttk.Scrollbar(panel, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scroll.set)
        scroll.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        inner = tk.Frame(canvas, bg=PANEL)
        window = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _fit(event):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfigure(window, width=event.width)

        inner.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", _fit)
        canvas.bind("<Enter>", lambda _e: canvas.bind_all("<MouseWheel>", _wheel))
        canvas.bind("<Leave>", lambda _e: canvas.unbind_all("<MouseWheel>"))

        def _wheel(event):
            canvas.yview_scroll(int(-event.delta / 120), "units")

        self._section(inner, "Serial")
        row = tk.Frame(inner, bg=PANEL)
        row.pack(fill="x", padx=16, pady=(0, 8))
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(row, textvariable=self.port_var, state="readonly", width=16)
        self.port_box.pack(side="left", fill="x", expand=True)
        ttk.Button(row, text="Refresh", command=self.refresh_ports).pack(side="left", padx=(8, 0))
        self.connect_btn = ttk.Button(inner, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(fill="x", padx=16, pady=(0, 6))
        tk.Label(
            inner, text="Upload stabilize.ino, 115200 baud",
            bg=PANEL, fg=MUTED, font=("Segoe UI", 9),
        ).pack(anchor="w", padx=16)

        self._section(inner, "Attitude")
        self.pitch_var = tk.StringVar(value="0.0°")
        self.roll_var = tk.StringVar(value="0.0°")
        self.yaw_var = tk.StringVar(value="0.0°")
        self._readout(inner, "Pitch", self.pitch_var)
        self._readout(inner, "Roll", self.roll_var)
        self._readout(inner, "Yaw", self.yaw_var)
        self.error_var = tk.StringVar(value="Pitch err 0.0°    Roll err 0.0°")
        tk.Label(inner, textvariable=self.error_var, bg=PANEL, fg=ACCENT, font=("Segoe UI", 9)).pack(
            anchor="w", padx=16, pady=(4, 0)
        )

        self._section(inner, "Stabilize")
        self.enable_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            inner, text="Hold target with IMU", variable=self.enable_var,
            command=self._on_enable,
        ).pack(anchor="w", padx=16)
        button_row = tk.Frame(inner, bg=PANEL)
        button_row.pack(fill="x", padx=16, pady=(8, 0))
        ttk.Button(button_row, text="Level", command=self.level).pack(side="left", expand=True, fill="x")
        ttk.Button(button_row, text="Hold this angle", command=self.hold_angle).pack(
            side="left", expand=True, fill="x", padx=(8, 0)
        )
        pitch_row = tk.Frame(inner, bg=PANEL)
        pitch_row.pack(fill="x", padx=16, pady=(8, 0))
        ttk.Button(pitch_row, text="Pitch up 5°", command=lambda: self.set_pose(PITCH_UP_DEG)).pack(
            side="left", expand=True, fill="x"
        )
        ttk.Button(pitch_row, text="Pitch Down 5°", command=lambda: self.set_pose(PITCH_DOWN_DEG)).pack(
            side="left", expand=True, fill="x", padx=(8, 0)
        )
        roll_row = tk.Frame(inner, bg=PANEL)
        roll_row.pack(fill="x", padx=16, pady=(8, 0))
        ttk.Button(roll_row, text="Roll left 5°", command=lambda: self.set_pose(ROLL_LEFT_DEG)).pack(
            side="left", expand=True, fill="x"
        )
        ttk.Button(roll_row, text="Roll Right 5°", command=lambda: self.set_pose(ROLL_RIGHT_DEG)).pack(
            side="left", expand=True, fill="x", padx=(8, 0)
        )

        self.target_pitch = tk.DoubleVar(value=0.0)
        self.target_roll = tk.DoubleVar(value=0.0)
        self._scale(inner, "Target pitch", self.target_pitch, -30, 30, 0.5, "°", self._queue_config)
        self._scale(inner, "Target roll", self.target_roll, -30, 30, 0.5, "°", self._queue_config)

        self._section(inner, "PID")
        self.kp = tk.DoubleVar(value=1.0)
        self.ki = tk.DoubleVar(value=0.0)
        self.kd = tk.DoubleVar(value=0.05)
        self._scale(inner, "Kp", self.kp, 0, 4, 0.05, "", self._queue_config)
        self._scale(inner, "Ki", self.ki, 0, 1, 0.01, "", self._queue_config)
        self._scale(inner, "Kd", self.kd, 0, 1, 0.01, "", self._queue_config)

        self._section(inner, "Mix  µs per degree")
        self.pitch_us = [tk.IntVar(value=15), tk.IntVar(value=15)]
        self.roll_us = [tk.IntVar(value=15), tk.IntVar(value=-15)]
        self._scale(inner, "Pitch servo 1", self.pitch_us[0], -80, 80, 1, "", self._queue_config)
        self._scale(inner, "Pitch servo 2", self.pitch_us[1], -80, 80, 1, "", self._queue_config)
        self._scale(inner, "Roll servo 1", self.roll_us[0], -80, 80, 1, "", self._queue_config)
        self._scale(inner, "Roll servo 2", self.roll_us[1], -80, 80, 1, "", self._queue_config)
        tk.Label(
            inner,
            text="If the table runs away from the target, flip the sign of that mix.",
            bg=PANEL, fg=MUTED, font=("Segoe UI", 9), wraplength=310, justify="left",
        ).pack(anchor="w", padx=16, pady=(0, 4))

        self._section(inner, "Motion")
        self.slew = tk.IntVar(value=120)
        self._scale(inner, "Slew", self.slew, 40, 400, 10, " µs/s", self._queue_config)
        self.trim = [tk.IntVar(value=1500), tk.IntVar(value=1500)]
        self.out_labels = []
        for index in (0, 1):
            self._scale(
                inner, "Trim servo %d" % (index + 1), self.trim[index],
                1000, 2000, 10, " µs", lambda i=index: self._queue_trim(i + 1),
            )
            label = tk.StringVar(value="Output 1500 µs")
            self.out_labels.append(label)
            tk.Label(inner, textvariable=label, bg=PANEL, fg=ACCENT, font=("Segoe UI", 9)).pack(anchor="w", padx=16)

        tk.Label(
            inner,
            text="Trim is the center the loop holds. With stabilize off, trim still slews the horn to that pulse.",
            bg=PANEL, fg=MUTED, font=("Segoe UI", 9), wraplength=310, justify="left",
        ).pack(anchor="w", padx=16, pady=(8, 12))

        self.status_var = tk.StringVar(value="Disconnected")
        tk.Label(
            inner, textvariable=self.status_var, bg=PANEL, fg=ACCENT,
            font=("Segoe UI", 10), wraplength=310, justify="left",
        ).pack(anchor="w", padx=16, pady=(0, 18))

    def _section(self, parent, title):
        tk.Label(
            parent, text=title, bg=PANEL, fg=TEXT, font=("Segoe UI", 13, "bold"),
        ).pack(anchor="w", padx=16, pady=(16, 8))

    def _readout(self, parent, name, variable):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", padx=16, pady=2)
        tk.Label(row, text=name, bg=PANEL, fg=MUTED, font=("Segoe UI", 11), width=8, anchor="w").pack(side="left")
        tk.Label(row, textvariable=variable, bg=PANEL, fg=TEXT, font=("Segoe UI", 16)).pack(side="left")

    def _scale(self, parent, title, variable, start, end, resolution, suffix, callback):
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", padx=16, pady=(8, 0))
        tk.Label(row, text=title, bg=PANEL, fg=TEXT, font=("Segoe UI", 10)).pack(side="left")
        value = tk.StringVar()
        tk.Label(row, textvariable=value, bg=PANEL, fg=ACCENT, font=("Segoe UI", 10)).pack(side="right")

        def show(raw):
            number = float(raw)
            if resolution >= 1:
                text = "%d%s" % (int(round(number)), suffix)
            else:
                text = "%0.2f%s" % (number, suffix)
            value.set(text)
            if self._ui_ready:
                callback()

        show(variable.get())
        tk.Scale(
            parent, from_=start, to=end, orient="horizontal", resolution=resolution,
            showvalue=False, variable=variable, command=show,
            bg=ACCENT, troughcolor="#0e141b", activebackground="#6cb2ff",
            highlightthickness=0, bd=0, sliderrelief="flat", sliderlength=22, width=14,
        ).pack(fill="x", padx=16)

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
        self._send_trim(1)
        self._send_trim(2)
        self._send_config()

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

    def _write(self, packet):
        if not self.running or self.serial is None:
            return
        try:
            with self.io_lock:
                self.serial.write(packet)
        except Exception:
            self.after(0, self._on_link_lost)

    def _queue_config(self):
        if not self._ui_ready:
            return
        if self._cfg_job is not None:
            self.after_cancel(self._cfg_job)
        self._cfg_job = self.after(80, self._send_config)

    def _send_config(self):
        self._cfg_job = None
        mix = (
            int(self.pitch_us[0].get()),
            int(self.pitch_us[1].get()),
            int(self.roll_us[0].get()),
            int(self.roll_us[1].get()),
        )
        packet = stab_packet(
            self.enable_var.get(),
            float(self.kp.get()),
            float(self.ki.get()),
            float(self.kd.get()),
            mix,
            float(self.target_pitch.get()),
            float(self.target_roll.get()),
            int(self.slew.get()),
        )
        self._write(packet)

    def _on_enable(self):
        if not self.enable_var.get():
            self._ignore_trim = True
            try:
                for index in (0, 1):
                    self.trim[index].set(int(self.parser.servo_us[index]))
            finally:
                self._ignore_trim = False
        self._queue_config()

    def _queue_trim(self, servo_id):
        if not self._ui_ready or self._ignore_trim:
            return
        self._pending_trim.add(servo_id)
        if self._trim_job is None:
            self._trim_job = self.after(40, self._flush_trim)

    def _flush_trim(self):
        self._trim_job = None
        pending = list(self._pending_trim)
        self._pending_trim.clear()
        for servo_id in pending:
            self._send_trim(servo_id)

    def _send_trim(self, servo_id):
        pulse = int(self.trim[servo_id - 1].get())
        self._write(servo_packet(servo_id, pulse))

    def _clamp_target(self, angle):
        snapped = round(float(angle) * 2.0) / 2.0
        return max(-30.0, min(30.0, snapped))

    def level(self):
        self.target_pitch.set(0.0)
        self.target_roll.set(0.0)
        self.enable_var.set(True)
        self._send_config()

    def hold_angle(self):
        self.target_pitch.set(self._clamp_target(self.parser.pitch))
        self.target_roll.set(self._clamp_target(self.parser.roll))
        self.enable_var.set(True)
        self._send_config()

    def set_pose(self, pose):
        pitch, roll = pose
        self.target_pitch.set(self._clamp_target(pitch))
        self.target_roll.set(self._clamp_target(roll))
        self.enable_var.set(True)
        self._send_config()

    def tick(self):
        pitch, roll, yaw = self.parser.pitch, self.parser.roll, self.parser.yaw
        self.view.set_attitude(pitch, roll, yaw)
        self.horizon.set_attitude(pitch, roll, yaw)
        self.pitch_var.set("%0.1f°" % pitch)
        self.roll_var.set("%0.1f°" % roll)
        self.yaw_var.set("%0.1f°" % yaw)
        self.error_var.set(
            "Pitch err %+0.1f°    Roll err %+0.1f°" % (
                float(self.target_pitch.get()) - pitch,
                float(self.target_roll.get()) - roll,
            )
        )
        for index, label in enumerate(self.out_labels):
            label.set("Output %d µs" % self.parser.servo_us[index])
        if self.running:
            if self.parser.have_attitude and time.monotonic() - self.parser.last_attitude < 0.5:
                self.status_var.set(self.parser.last_ack or "Receiving attitude")
            elif self.parser.have_attitude:
                self.status_var.set("No recent attitude packet")
            else:
                self.status_var.set("Waiting for attitude (gyro calibration)")
        self.after(33, self.tick)

    def on_close(self):
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
