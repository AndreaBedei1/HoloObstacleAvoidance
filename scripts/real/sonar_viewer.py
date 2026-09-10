"""Dual live diagnostic viewer for the two real BlueROV2 sonars.

Left panel: Cerulean Omniscan 450 SS over TCP 192.168.2.86:51200.
Right panel: Blue Robotics Ping1D through BlueOS PingProxy UDP
192.168.2.2:9090.

The live path uses the official ``bluerobotics-ping`` package for both
devices.  Connecting does not start Omniscan pinging.  The only Omniscan
control command is sent after the user presses Start Omniscan, and a matching
enable=0 command is sent before a started session is disconnected or closed.

This program never writes BlueOS configuration, serial configuration, or
ROV network configuration.  It is a diagnostic viewer, not a vehicle-control
application.
"""

from __future__ import print_function

import argparse
import json
import os
import queue
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageDraw, ImageTk


OMNISCAN_HOST = "192.168.2.86"
OMNISCAN_PORT = 51200
PING1D_HOST = "192.168.2.2"
PING1D_PORT = 9090
BLUEOS_HOST = "192.168.2.2"

APP_ROOT = Path(__file__).resolve().parents[2]
RECORD_ROOT = APP_ROOT / "records" / "sonar"

try:
    from brping import Omniscan450, Ping1D, definitions
    BRPING_IMPORT_ERROR = None
except ImportError as exc:  # Keep the offline parser/GUI importable pre-install.
    Omniscan450 = None
    Ping1D = None
    definitions = None
    BRPING_IMPORT_ERROR = exc


# These definitions are kept for offline parser tests and for validating the
# packet shape against the Cerulean/Ping Protocol.  Live Omniscan I/O uses the
# official Omniscan450 class above.
PACKET_HEADER = struct.Struct("<BBHHBB")
PACKET_CHECKSUM = struct.Struct("<H")
OS_PING_PARAMS = 2197
OS_MONO_PROFILE = 2198
OMNI_PROFILE_HEADER = struct.Struct("<IIIIIHHHBBffffff")


def make_packet(packet_id, payload):
    """Build a Cerulean/Ping Protocol packet for offline tests."""
    header = PACKET_HEADER.pack(66, 82, len(payload), packet_id, 0, 0)
    return header + payload + PACKET_CHECKSUM.pack(sum(header + payload) & 0xFFFF)


def decode_omni_profile_payload(payload):
    """Decode a raw os_mono_profile payload for offline validation.

    The live viewer receives decoded PingMessage objects from brping.  This
    helper makes the packet parser independently testable without hardware.
    """
    if len(payload) < OMNI_PROFILE_HEADER.size:
        raise ValueError("short os_mono_profile payload")
    values = OMNI_PROFILE_HEADER.unpack_from(payload)
    (
        ping_number,
        start_mm,
        length_mm,
        timestamp_ms,
        acoustic_frequency_hz,
        gain_index,
        num_results,
        sos_dmps,
        channel_number,
        _reserved,
        pulse_duration_sec,
        analog_gain,
        max_pwr_db,
        min_pwr_db,
        transducer_heading_deg,
        vehicle_heading_deg,
    ) = values
    available = (len(payload) - OMNI_PROFILE_HEADER.size) // 2
    count = min(int(num_results), available)
    raw = list(struct.unpack_from("<%dH" % count, payload, OMNI_PROFILE_HEADER.size))
    return {
        "ping_number": ping_number,
        "start_mm": start_mm,
        "length_mm": length_mm,
        "timestamp_ms": timestamp_ms,
        "acoustic_frequency_hz": acoustic_frequency_hz,
        "gain_index": gain_index,
        "num_results": count,
        "sos_dmps": sos_dmps,
        "channel_number": channel_number,
        "pulse_duration_sec": pulse_duration_sec,
        "analog_gain": analog_gain,
        "max_pwr_db": max_pwr_db,
        "min_pwr_db": min_pwr_db,
        "transducer_heading_deg": transducer_heading_deg,
        "vehicle_heading_deg": vehicle_heading_deg,
        "pwr_results": raw,
    }


def normalized_row(values, low=None, high=None):
    """Convert an iterable of values into 0..255 display intensities."""
    values = list(values or [])
    if not values:
        return []
    if low is None:
        low = min(values)
    if high is None:
        high = max(values)
    low = float(low)
    high = float(high)
    if high <= low:
        return [0 for _ in values]
    return [
        max(0, min(255, int(round((float(value) - low) * 255.0 / (high - low)))))
        for value in values
    ]


def colorize(row):
    """Blue-to-yellow sonar palette as RGB bytes."""
    out = bytearray(len(row) * 3)
    for index, value in enumerate(row):
        x = float(value) / 255.0
        red = int(255 * max(0.0, min(1.0, (x - 0.42) * 2.2)))
        green = int(255 * max(0.0, min(1.0, (x - 0.18) * 1.55)))
        blue = int(255 * max(0.0, min(1.0, 0.25 + x * 0.9)))
        offset = index * 3
        out[offset : offset + 3] = bytes((red, green, blue))
    return bytes(out)


def close_brping_device(device):
    """Close a brping device without assuming a particular package version."""
    if device is None:
        return
    io_device = getattr(device, "iodev", None)
    if io_device is not None:
        try:
            io_device.close()
        except Exception:
            pass


def omni_control(device, *, enable, range_mm=20000, points=600, gain=-1, ping_hz=8.0):
    """Send Omniscan settings only when explicitly requested by the user."""
    kwargs = {"enable": int(bool(enable))}
    if enable:
        kwargs.update(
            {
                "start_mm": 0,
                "length_mm": int(range_mm),
                "msec_per_ping": int(round(1000.0 / float(ping_hz))),
                "gain_index": int(gain),
                "num_results": int(points),
            }
        )
    try:
        device.control_os_ping_params(**kwargs)
    except TypeError:
        # Older official package builds did not expose num_results as a
        # keyword.  Keep the explicit start/stop behavior and retry safely.
        kwargs.pop("num_results", None)
        device.control_os_ping_params(**kwargs)


def omni_profile_record(device, previous_arrival):
    """Convert an official Omniscan PingMessage into display/record data."""
    scaled = list(Omniscan450.scale_power(device))
    if not scaled:
        return None, previous_arrival
    now = time.time()
    rate = 0.0 if previous_arrival is None else 1.0 / max(1e-6, now - previous_arrival)
    min_db = getattr(device, "min_pwr_db", None)
    max_db = getattr(device, "max_pwr_db", None)
    row = normalized_row(scaled, min_db, max_db)
    record = {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "ping_number": int(getattr(device, "ping_number", 0)),
        "start_mm": int(getattr(device, "start_mm", 0)),
        "length_mm": int(getattr(device, "length_mm", 0)),
        "range_m": float(getattr(device, "length_mm", 0)) / 1000.0,
        "ping_rate_hz": rate,
        # The protocol field is the acoustic carrier frequency (normally
        # 450000), not the temporal ping rate.  Keep both unambiguous.
        "acoustic_frequency_hz": int(getattr(device, "ping_hz", 0)),
        "gain": int(getattr(device, "gain_index", -1)),
        "min_pwr_db": float(min_db) if min_db is not None else None,
        "max_pwr_db": float(max_db) if max_db is not None else None,
        "profile_db": [float(value) for value in scaled],
        "display_row": row,
    }
    return record, now


def ping1d_profile_record(profile, distance_record):
    """Normalize an official Ping1D profile and mark the selected distance."""
    if not profile:
        return None
    data = list(profile.get("data", []))
    start_mm = int(profile.get("scan_start", 0))
    length_mm = int(profile.get("scan_length", 0))
    distance_mm = int((distance_record or {}).get("distance", profile.get("distance", 0)))
    row = normalized_row(data, 0, 255)
    return {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "distance_mm": distance_mm,
        "distance_m": distance_mm / 1000.0,
        "confidence": int((distance_record or {}).get("confidence", profile.get("confidence", 0))),
        "scan_start_mm": start_mm,
        "scan_length_mm": length_mm,
        "gain": int(profile.get("gain_index", -1)),
        "profile": [int(value) for value in data],
        "display_row": row,
    }


class OmniscanWorker(threading.Thread):
    """Official brping Omniscan worker; idle after connect until Start."""

    def __init__(self, host, port, events):
        super(OmniscanWorker, self).__init__(daemon=True)
        self.host = host
        self.port = port
        self.events = events
        self.commands = queue.Queue()
        self.stop_event = threading.Event()
        self.device = None
        self.started_by_app = False

    def command(self, name, payload=None):
        self.commands.put((name, payload))

    def _handle_commands(self):
        handled = False
        while True:
            try:
                name, payload = self.commands.get_nowait()
            except queue.Empty:
                return handled
            handled = True
            if name == "start":
                omni_control(self.device, enable=True, **payload)
                self.started_by_app = True
                self.events.put(("omni_started", None))
            elif name == "stop":
                if self.started_by_app:
                    omni_control(self.device, enable=False)
                self.started_by_app = False
                self.events.put(("omni_stopped", None))
            elif name == "disconnect":
                self.stop_event.set()
                return handled

    def run(self):
        previous_arrival = None
        try:
            if Omniscan450 is None:
                raise RuntimeError("bluerobotics-ping non installato")
            self.device = Omniscan450()
            self.device.connect_tcp(self.host, self.port)
            if self.device.initialize() is False:
                raise RuntimeError("Omniscan initialize() fallita")
            self.events.put(("omni_connected", None))
            while not self.stop_event.is_set():
                self._handle_commands()
                if self.stop_event.is_set():
                    break
                if not self.started_by_app:
                    time.sleep(0.05)
                    continue
                data = self.device.wait_message([definitions.OMNISCAN450_OS_MONO_PROFILE])
                if data:
                    record, previous_arrival = omni_profile_record(data, previous_arrival)
                    if record:
                        self.events.put(("omni_profile", record))
        except Exception as exc:
            self.events.put(("omni_error", str(exc)))
        finally:
            if self.device is not None:
                if self.started_by_app:
                    try:
                        omni_control(self.device, enable=False)
                    except Exception:
                        pass
                    self.started_by_app = False
                close_brping_device(self.device)
            self.events.put(("omni_closed", None))


class Ping1DWorker(threading.Thread):
    """Official brping Ping1D worker through BlueOS PingProxy UDP."""

    def __init__(self, host, port, events):
        super(Ping1DWorker, self).__init__(daemon=True)
        self.host = host
        self.port = port
        self.events = events
        self.stop_event = threading.Event()
        self.device = None

    def run(self):
        try:
            if Ping1D is None:
                raise RuntimeError("bluerobotics-ping non installato")
            self.device = Ping1D()
            self.device.connect_udp(self.host, self.port)
            if self.device.initialize() is False:
                raise RuntimeError("Ping1D initialize() fallita")
            self.events.put(("ping_connected", None))
            while not self.stop_event.is_set():
                distance = self.device.get_distance()
                if not distance:
                    self.events.put(("ping_warning", "get_distance() senza risposta"))
                    time.sleep(0.1)
                    continue
                profile = None
                try:
                    profile = self.device.get_profile()
                except Exception:
                    # Some older Ping1D firmware/proxy combinations expose
                    # distance but not profile; keep distance diagnostics alive.
                    profile = None
                record = {
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "distance_mm": int(distance.get("distance", 0)),
                    "distance_m": float(distance.get("distance", 0)) / 1000.0,
                    "confidence": int(distance.get("confidence", 0)),
                }
                profile_record = ping1d_profile_record(profile, distance)
                self.events.put(("ping_sample", (record, profile_record)))
                time.sleep(0.02)
        except Exception as exc:
            self.events.put(("ping_error", str(exc)))
        finally:
            close_brping_device(self.device)
            self.events.put(("ping_closed", None))

    def stop(self):
        self.stop_event.set()
        close_brping_device(self.device)


class BlueOSWorker(threading.Thread):
    """Read-only reachability probe for BlueOS HTTP and mavlink2rest ports."""

    def __init__(self, host, events):
        super(BlueOSWorker, self).__init__(daemon=True)
        self.host = host
        self.events = events
        self.stop_event = threading.Event()

    @staticmethod
    def _check(host, port):
        try:
            sock = socket.create_connection((host, port), 1.0)
            sock.close()
            return True
        except OSError:
            return False

    def run(self):
        while not self.stop_event.is_set():
            online = self._check(self.host, 80) or self._check(self.host, 6040)
            self.events.put(("blueos", online))
            self.stop_event.wait(2.0)


class SonarViewer(tk.Tk):
    OMNI_W = 570
    OMNI_H = 300
    PROFILE_W = 570
    PROFILE_H = 210

    def __init__(self):
        super(SonarViewer, self).__init__()
        self.title("BlueROV2 — dual sonar diagnostics")
        self.geometry("1280x900")
        self.minsize(1100, 760)
        self.events = queue.Queue()
        self.omni_worker = None
        self.ping_worker = None
        self.blueos_worker = None
        self.omni_started = False
        self.omni_rows = deque(maxlen=self.OMNI_H)
        self.omni_latest = None
        self.ping_latest = None
        self.ping_profile_latest = None
        self.photos = {}
        self.record_file = None
        self.record_dir = None
        self.recording = False

        self.omni_range_m = tk.DoubleVar(value=20.0)
        self.omni_points = tk.IntVar(value=600)
        self.omni_gain = tk.IntVar(value=-1)
        self.omni_ping_hz = tk.DoubleVar(value=8.0)
        self.status_text = tk.StringVar(value="Pronto — nessun dispositivo collegato")
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(50, self._poll_events)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        self.blueos_badge = self._badge(top, "BlueOS", BLUEOS_HOST, 0)
        self.omni_badge = self._badge(top, "Omniscan", OMNISCAN_HOST, 1)
        self.ping_badge = self._badge(top, "Ping1D", "%s:%d" % (PING1D_HOST, PING1D_PORT), 2)
        for column in range(3):
            top.columnconfigure(column, weight=1)

        toolbar = ttk.Frame(self, padding=(8, 0, 8, 8))
        toolbar.pack(fill="x")
        ttk.Button(toolbar, text="Connect all", command=self.connect_all).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Disconnect", command=self.disconnect_all).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Start Omniscan", command=self.start_omni).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Stop Omniscan", command=self.stop_omni).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Save screenshot", command=self.save_screenshot).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Start recording", command=self.start_recording).pack(side="left", padx=2)
        ttk.Button(toolbar, text="Stop recording", command=self.stop_recording).pack(side="left", padx=2)
        ttk.Label(toolbar, textvariable=self.status_text).pack(side="right", padx=4)

        panes = ttk.Panedwindow(self, orient="horizontal")
        panes.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        left = ttk.Frame(panes, padding=8)
        right = ttk.Frame(panes, padding=8)
        panes.add(left, weight=1)
        panes.add(right, weight=1)
        self._build_omni_panel(left)
        self._build_ping_panel(right)

    def _badge(self, parent, title, address, column):
        frame = ttk.LabelFrame(parent, text=title, padding=6)
        frame.grid(row=0, column=column, sticky="ew", padx=3)
        state = tk.StringVar(value="OFFLINE")
        ttk.Label(frame, text=address).pack(side="left")
        label = ttk.Label(frame, textvariable=state, foreground="#b00020")
        label.pack(side="right")
        return {"state": state, "label": label}

    def _build_omni_panel(self, parent):
        ttk.Label(parent, text="OMNISCAN 450 SS", font=("Segoe UI", 14, "bold")).pack(anchor="w")
        controls = ttk.LabelFrame(parent, text="Start parameters", padding=6)
        controls.pack(fill="x", pady=(6, 6))
        ttk.Label(controls, text="Range (m)").grid(row=0, column=0, sticky="w")
        ttk.Entry(controls, textvariable=self.omni_range_m, width=8).grid(row=0, column=1, padx=(4, 10))
        ttk.Label(controls, text="Points").grid(row=0, column=2, sticky="w")
        ttk.Entry(controls, textvariable=self.omni_points, width=8).grid(row=0, column=3, padx=(4, 10))
        ttk.Label(controls, text="Gain (-1 auto)").grid(row=0, column=4, sticky="w")
        ttk.Entry(controls, textvariable=self.omni_gain, width=8).grid(row=0, column=5, padx=(4, 10))
        ttk.Label(controls, text="Ping/s").grid(row=0, column=6, sticky="w")
        ttk.Entry(controls, textvariable=self.omni_ping_hz, width=8).grid(row=0, column=7, padx=(4, 0))

        self.omni_waterfall_label = ttk.Label(parent, text="Connect all, poi Start Omniscan")
        self.omni_waterfall_label.pack(fill="both", expand=True)
        self.omni_profile_label = ttk.Label(parent, text="Ultimo profilo intensity-vs-range")
        self.omni_profile_label.pack(fill="both", expand=True, pady=(6, 0))
        self.omni_stats = tk.StringVar(value="Range: — | ping rate: — | gain: — | ping: —")
        ttk.Label(parent, textvariable=self.omni_stats, anchor="w").pack(fill="x", pady=(5, 0))

    def _build_ping_panel(self, parent):
        ttk.Label(parent, text="PING1D via BlueOS PingProxy", font=("Segoe UI", 14, "bold")).pack(anchor="w")
        self.distance_label = ttk.Label(parent, text="DISTANCE: —", font=("Segoe UI", 25, "bold"), foreground="#444444")
        self.distance_label.pack(anchor="w", pady=(10, 0))
        self.confidence_label = ttk.Label(parent, text="CONFIDENCE: — %", font=("Segoe UI", 18, "bold"))
        self.confidence_label.pack(anchor="w")
        self.ping_profile_label = ttk.Label(parent, text="Profilo echi Ping1D non ancora ricevuto")
        self.ping_profile_label.pack(fill="both", expand=True, pady=(15, 0))
        self.ping_stats = tk.StringVar(value="Profilo: — | scan range: — | gain: —")
        ttk.Label(parent, textvariable=self.ping_stats, anchor="w").pack(fill="x", pady=(5, 0))

    def _set_badge(self, badge, online, detail=None):
        text = "ONLINE" if online else "OFFLINE"
        if detail:
            text += " — " + detail
        badge["state"].set(text)
        badge["label"].configure(foreground="#087f23" if online else "#b00020")

    def _omni_params(self):
        range_mm = int(round(float(self.omni_range_m.get()) * 1000.0))
        points = int(self.omni_points.get())
        gain = int(self.omni_gain.get())
        ping_hz = float(self.omni_ping_hz.get())
        if not 1 <= range_mm <= 150000:
            raise ValueError("Omniscan range deve essere 0.001..150 m")
        if not 200 <= points <= 1200:
            raise ValueError("Omniscan points deve essere 200..1200")
        if not -1 <= gain <= 7:
            raise ValueError("Omniscan gain deve essere -1 oppure 0..7")
        if not 0.2 <= ping_hz <= 20:
            raise ValueError("Omniscan ping/s deve essere 0.2..20")
        return {"range_mm": range_mm, "points": points, "gain": gain, "ping_hz": ping_hz}

    def connect_all(self):
        if self.blueos_worker is None:
            self.blueos_worker = BlueOSWorker(BLUEOS_HOST, self.events)
            self.blueos_worker.start()
        if self.omni_worker is None:
            self.omni_worker = OmniscanWorker(OMNISCAN_HOST, OMNISCAN_PORT, self.events)
            self.omni_worker.start()
        if self.ping_worker is None:
            self.ping_worker = Ping1DWorker(PING1D_HOST, PING1D_PORT, self.events)
            self.ping_worker.start()
        self.status_text.set("Connessioni in avvio — Omniscan resta fermo fino a Start Omniscan")

    def start_omni(self):
        if self.omni_worker is None:
            messagebox.showwarning("Omniscan", "Premi prima Connect all.")
            return
        try:
            self.omni_worker.command("start", self._omni_params())
            self.status_text.set("Start Omniscan richiesto")
        except Exception as exc:
            messagebox.showerror("Parametri Omniscan", str(exc))

    def stop_omni(self):
        if self.omni_worker is not None:
            self.omni_worker.command("stop")
        self.status_text.set("Stop Omniscan richiesto")

    def disconnect_all(self):
        self.stop_omni()
        if self.omni_worker is not None:
            self.omni_worker.command("disconnect")
        if self.ping_worker is not None:
            self.ping_worker.stop()
        if self.blueos_worker is not None:
            self.blueos_worker.stop_event.set()
        self.status_text.set("Disconnessione richiesta")

    def _poll_events(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == "blueos":
                    self._set_badge(self.blueos_badge, data)
                elif kind == "omni_connected":
                    self._set_badge(self.omni_badge, True)
                    self.status_text.set("Omniscan connesso — ping fermo")
                elif kind == "omni_started":
                    self.omni_started = True
                    self.status_text.set("Omniscan attivo")
                elif kind == "omni_stopped":
                    self.omni_started = False
                    self.status_text.set("Omniscan fermato")
                elif kind == "omni_profile":
                    self.omni_latest = data
                    self._update_omni(data)
                    self._record_snapshot()
                elif kind == "omni_error":
                    self._set_badge(self.omni_badge, False, data)
                    self.status_text.set("Errore Omniscan: %s" % data)
                elif kind == "omni_closed":
                    self.omni_worker = None
                    self.omni_started = False
                    self._set_badge(self.omni_badge, False)
                elif kind == "ping_connected":
                    self._set_badge(self.ping_badge, True)
                elif kind == "ping_sample":
                    distance, profile = data
                    self.ping_latest = distance
                    if profile:
                        self.ping_profile_latest = profile
                    self._update_ping(distance, profile)
                    self._record_snapshot()
                elif kind == "ping_warning":
                    self._set_badge(self.ping_badge, True, "no data")
                elif kind == "ping_error":
                    self._set_badge(self.ping_badge, False, data)
                    self.status_text.set("Errore Ping1D: %s" % data)
                elif kind == "ping_closed":
                    self.ping_worker = None
                    self._set_badge(self.ping_badge, False)
        except queue.Empty:
            pass
        self.after(50, self._poll_events)

    def _update_omni(self, record):
        row = colorize(record["display_row"])
        if len(record["display_row"]) != self.OMNI_W:
            line = Image.frombytes("RGB", (len(record["display_row"]), 1), row)
            line = line.resize((self.OMNI_W, 1), Image.Resampling.BILINEAR)
            row = line.tobytes()
        self.omni_rows.append(row)
        while len(self.omni_rows) < self.OMNI_H:
            self.omni_rows.appendleft(bytes(self.OMNI_W * 3))
        waterfall = Image.frombytes("RGB", (self.OMNI_W, self.OMNI_H), b"".join(self.omni_rows))
        self.photos["omni_waterfall"] = ImageTk.PhotoImage(waterfall)
        self.omni_waterfall_label.configure(image=self.photos["omni_waterfall"], text="")
        profile = self._plot_profile(
            record["profile_db"],
            0.0,
            record["range_m"],
            None,
            "Omniscan intensity (dB) vs range (m)",
            "dB",
        )
        self.photos["omni_profile"] = ImageTk.PhotoImage(profile)
        self.omni_profile_label.configure(image=self.photos["omni_profile"], text="")
        self.omni_stats.set(
            "Range: %.2f m | ping rate: %.2f Hz | gain: %d | ping: %d"
            % (record["range_m"], record["ping_rate_hz"], record["gain"], record["ping_number"])
        )

    def _update_ping(self, distance, profile):
        distance_m = distance["distance_m"]
        confidence = distance["confidence"]
        self.distance_label.configure(text="DISTANCE: %.2f m" % distance_m, foreground="#087f23")
        self.confidence_label.configure(text="CONFIDENCE: %d %%" % confidence)
        if not profile:
            return
        plot = self._plot_profile(
            profile["profile"],
            profile["scan_start_mm"] / 1000.0,
            (profile["scan_start_mm"] + profile["scan_length_mm"]) / 1000.0,
            distance_m,
            "Ping1D echo profile (selected distance marked)",
            "return strength",
        )
        self.photos["ping_profile"] = ImageTk.PhotoImage(plot)
        self.ping_profile_label.configure(image=self.photos["ping_profile"], text="")
        self.ping_stats.set(
            "Profilo: %d campioni | scan range: %.2f..%.2f m | gain: %d"
            % (
                len(profile["profile"]),
                profile["scan_start_mm"] / 1000.0,
                (profile["scan_start_mm"] + profile["scan_length_mm"]) / 1000.0,
                profile["gain"],
            )
        )

    def _plot_profile(self, values, x_min, x_max, marker, title, y_label):
        image = Image.new("RGB", (self.PROFILE_W, self.PROFILE_H), "#101820")
        draw = ImageDraw.Draw(image)
        left, top, right, bottom = 52, 26, self.PROFILE_W - 12, self.PROFILE_H - 28
        draw.text((8, 5), title, fill="white")
        draw.line((left, top, left, bottom), fill="#aaaaaa")
        draw.line((left, bottom, right, bottom), fill="#aaaaaa")
        values = list(values or [])
        if not values:
            draw.text((left + 10, top + 10), "nessun profilo", fill="#dddddd")
            return image
        low = min(values)
        high = max(values)
        if high <= low:
            high = low + 1.0
        points = []
        for index, value in enumerate(values):
            fraction = 0.0 if len(values) == 1 else float(index) / (len(values) - 1)
            x = left + fraction * (right - left)
            y = bottom - (float(value) - low) / (high - low) * (bottom - top)
            points.append((x, y))
        if len(points) > 1:
            draw.line(points, fill="#38d9ff", width=2)
        if marker is not None and x_max > x_min:
            marker_fraction = max(0.0, min(1.0, (marker - x_min) / (x_max - x_min)))
            marker_x = left + marker_fraction * (right - left)
            draw.line((marker_x, top, marker_x, bottom), fill="#ffcc33", width=2)
            draw.text((max(left, marker_x - 24), top + 3), "%.2fm" % marker, fill="#ffcc33")
        draw.text((left, bottom + 5), "%.2f m" % x_min, fill="#cccccc")
        draw.text((right - 48, bottom + 5), "%.2f m" % x_max, fill="#cccccc")
        draw.text((4, bottom - 12), y_label, fill="#cccccc")
        return image

    def _record_snapshot(self):
        if not self.recording or self.record_file is None:
            return
        item = {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "ping1d": self.ping_latest,
            "ping1d_profile": self.ping_profile_latest,
            "omniscan": self.omni_latest,
        }
        try:
            self.record_file.write(json.dumps(item, ensure_ascii=False) + "\n")
            self.record_file.flush()
        except OSError as exc:
            self.status_text.set("Errore registrazione: %s" % exc)

    def start_recording(self):
        if self.recording:
            return
        self.record_dir = RECORD_ROOT / time.strftime("%Y%m%d_%H%M%S")
        self.record_dir.mkdir(parents=True, exist_ok=True)
        self.record_file = open(str(self.record_dir / "sonar_frames.jsonl"), "w", encoding="utf-8")
        metadata = {
            "started": datetime.now().isoformat(),
            "blueos": BLUEOS_HOST,
            "omniscan": "%s:%d" % (OMNISCAN_HOST, OMNISCAN_PORT),
            "ping1d": "%s:%d" % (PING1D_HOST, PING1D_PORT),
        }
        self.record_file.write(json.dumps({"metadata": metadata}) + "\n")
        self.record_file.flush()
        self.recording = True
        self.status_text.set("Registrazione: %s" % self.record_dir)

    def stop_recording(self):
        if self.record_file is not None:
            self.record_file.close()
        self.record_file = None
        self.recording = False
        self.status_text.set("Registrazione fermata")

    def _snapshot_image(self):
        canvas = Image.new("RGB", (1200, 760), "#202830")
        draw = ImageDraw.Draw(canvas)
        draw.text((20, 12), "BlueROV2 dual sonar — %s" % datetime.now().isoformat(timespec="seconds"), fill="white")
        # Use the last PIL renderings from the data rather than Tk internals.
        if self.omni_latest:
            row = colorize(self.omni_latest["display_row"])
            if len(self.omni_latest["display_row"]) != self.OMNI_W:
                line = Image.frombytes("RGB", (len(self.omni_latest["display_row"]), 1), row)
                line = line.resize((self.OMNI_W, 1), Image.Resampling.BILINEAR)
                row = line.tobytes()
            rows = list(self.omni_rows)
            while len(rows) < self.OMNI_H:
                rows.insert(0, bytes(self.OMNI_W * 3))
            canvas.paste(Image.frombytes("RGB", (self.OMNI_W, self.OMNI_H), b"".join(rows)), (20, 45))
            canvas.paste(self._plot_profile(self.omni_latest["profile_db"], 0, self.omni_latest["range_m"], None, "Omniscan profile", "dB"), (20, 365))
        if self.ping_profile_latest:
            p = self.ping_profile_latest
            canvas.paste(self._plot_profile(p["profile"], p["scan_start_mm"] / 1000.0, (p["scan_start_mm"] + p["scan_length_mm"]) / 1000.0, p["distance_m"], "Ping1D profile", "return"), (610, 45))
            draw.text((610, 285), "DISTANCE: %.2f m" % p["distance_m"], fill="white")
            draw.text((610, 310), "CONFIDENCE: %d %%" % p["confidence"], fill="white")
        return canvas

    def save_screenshot(self):
        path = filedialog.asksaveasfilename(
            title="Save sonar screenshot",
            defaultextension=".png",
            filetypes=[("PNG", "*.png")],
            initialfile="dual_sonar_%s.png" % time.strftime("%Y%m%d_%H%M%S"),
        )
        if path:
            self._snapshot_image().save(path)
            self.status_text.set("Screenshot salvato: %s" % os.path.basename(path))

    def close(self):
        self.stop_recording()
        self.disconnect_all()
        if self.omni_worker is not None:
            self.omni_worker.join(timeout=1.5)
        if self.ping_worker is not None:
            self.ping_worker.join(timeout=1.0)
        if self.blueos_worker is not None:
            self.blueos_worker.join(timeout=0.5)
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description="BlueROV2 dual sonar diagnostic GUI")
    parser.add_argument("--offline", action="store_true", help="open GUI without connecting")
    args = parser.parse_args()
    app = SonarViewer()
    if not args.offline:
        # Deliberately do not auto-connect or auto-start pinging.  The user
        # must press Connect all, and then Start Omniscan explicitly.
        pass
    app.mainloop()


if __name__ == "__main__":
    main()
