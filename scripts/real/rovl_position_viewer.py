"""BlueROV2 Position Tracker for a Cerulean ROV Locator Mk III.

Read-only serial application.  It never writes to the ROVL or GPS ports.
ROVL: 115200 8N1, ASCII/NMEA-like $USRTH sentences.
GPS: read-only NMEA input; baud is selectable and auto-detection tries common
rates.  Relative position works without GPS and without Internet.
"""

from __future__ import print_function

import argparse
import csv
import json
import os
import queue
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from PIL import ImageGrab

try:
    import serial
    from serial.tools import list_ports
    SERIAL_IMPORT_ERROR = None
except ImportError as exc:
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = exc

from rovl_protocol import classify_serial_sentence, forward_geodesic, parse_gps_sentence, parse_rovl_sentence, relative_position


ROVL_BAUD = 115200
GPS_BAUDS = (4800, 9600, 38400, 115200)
APP_ROOT = Path(__file__).resolve().parents[2]
RECORD_ROOT = APP_ROOT / "records" / "rovl"
CSV_FIELDS = [
    "timestamp", "raw", "topside_lat", "topside_lon", "rov_lat", "rov_lon",
    "slant_range_m", "horizontal_range_m", "bearing_deg", "elevation_deg",
    "east_m", "north_m",
]


def port_names():
    if list_ports is None:
        return []
    return [port.device for port in list_ports.comports()]


class ROVLSerialWorker(threading.Thread):
    def __init__(self, port, events):
        super(ROVLSerialWorker, self).__init__(daemon=True)
        self.port = port
        self.events = events
        self.stop_event = threading.Event()
        self.device = None

    def run(self):
        try:
            if serial is None:
                raise RuntimeError("pyserial non installato")
            self.device = serial.Serial(self.port, ROVL_BAUD, timeout=0.5)
            self.events.put(("rovl_connected", self.port))
            while not self.stop_event.is_set():
                line = self.device.readline()
                if not line:
                    continue
                try:
                    classified = classify_serial_sentence(line)
                except ValueError as exc:
                    self.events.put(("rovl_warning", str(exc)))
                    continue
                if classified is not None:
                    self.events.put(classified)
        except Exception as exc:
            self.events.put(("rovl_error", str(exc)))
        finally:
            if self.device is not None:
                try:
                    self.device.close()
                except Exception:
                    pass
            self.events.put(("rovl_closed", None))

    def stop(self):
        self.stop_event.set()
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass


class GPSSerialWorker(threading.Thread):
    def __init__(self, port, baud, events):
        super(GPSSerialWorker, self).__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.events = events
        self.stop_event = threading.Event()
        self.device = None

    def run(self):
        try:
            if serial is None:
                raise RuntimeError("pyserial non installato")
            self.device = serial.Serial(self.port, self.baud, timeout=0.5)
            self.events.put(("gps_connected", (self.port, self.baud)))
            while not self.stop_event.is_set():
                line = self.device.readline()
                if not line:
                    continue
                try:
                    message = parse_gps_sentence(line)
                except ValueError:
                    continue
                if message is not None:
                    self.events.put(("gps_message", message))
        except Exception as exc:
            self.events.put(("gps_error", str(exc)))
        finally:
            if self.device is not None:
                try:
                    self.device.close()
                except Exception:
                    pass
            self.events.put(("gps_closed", None))

    def stop(self):
        self.stop_event.set()
        if self.device is not None:
            try:
                self.device.close()
            except Exception:
                pass


def probe_port(port, baud, seconds, parser):
    """Read-only COM probe.  It never calls serial.write()."""
    if serial is None:
        return None
    device = None
    last = None
    try:
        device = serial.Serial(port, baud, timeout=0.15)
        deadline = time.time() + seconds
        while time.time() < deadline:
            line = device.readline()
            if not line:
                continue
            try:
                result = parser(line)
            except ValueError:
                continue
            if result is not None:
                last = result
                return last
        return last
    except Exception:
        return None
    finally:
        if device is not None:
            try:
                device.close()
            except Exception:
                pass


def probe_rovl_gps_port(port, baud, seconds):
    """Read one ROVL port once and retain both ROVL and GPS retweets."""
    result = {"rovl": None, "gps": None}
    if serial is None:
        return result
    device = None
    try:
        device = serial.Serial(port, baud, timeout=0.15)
        deadline = time.time() + seconds
        while time.time() < deadline:
            line = device.readline()
            if not line:
                continue
            try:
                classified = classify_serial_sentence(line)
            except ValueError:
                continue
            if classified is None:
                continue
            kind, message = classified
            if kind == "rovl_message":
                result["rovl"] = message
            elif kind == "gps_message":
                result["gps"] = message
    except Exception:
        pass
    finally:
        if device is not None:
            try:
                device.close()
            except Exception:
                pass
    return result


class PortScanWorker(threading.Thread):
    def __init__(self, events, seconds=1.2):
        super(PortScanWorker, self).__init__(daemon=True)
        self.events = events
        self.seconds = seconds

    def run(self):
        rovl_ports = []
        gps_ports = []
        gps_bauds = {}
        details = []
        for port in port_names():
            combined = probe_rovl_gps_port(port, ROVL_BAUD, self.seconds)
            rovl = combined["rovl"]
            retweeted_gps = combined["gps"]
            if rovl is not None:
                rovl_ports.append(port)
            if retweeted_gps is not None:
                gps_ports.append(port)
                gps_bauds[port] = ROVL_BAUD
            if rovl is not None and retweeted_gps is not None:
                details.append("%s: ROVL + GPS retweet @ %d" % (port, ROVL_BAUD))
            elif rovl is not None:
                details.append("%s: ROVL $USRTH" % port)
            elif retweeted_gps is not None:
                details.append("%s: GPS %d" % (port, ROVL_BAUD))
            found_gps = False
            if retweeted_gps is None:
                for baud in GPS_BAUDS:
                    if baud == ROVL_BAUD:
                        continue
                    gps = probe_port(port, baud, self.seconds, parse_gps_sentence)
                    if gps is not None:
                        if port not in gps_ports:
                            gps_ports.append(port)
                        gps_bauds[port] = baud
                        details.append("%s: GPS %d" % (port, baud))
                        found_gps = True
                        break
            if not found_gps and rovl is None and retweeted_gps is None:
                details.append("%s: nessun NMEA riconosciuto" % port)
        self.events.put(("scan_done", {"rovl": rovl_ports, "gps": gps_ports, "gps_bauds": gps_bauds, "details": details}))


class PositionTracker(tk.Tk):
    CANVAS_SIZE = 620

    def __init__(self):
        super(PositionTracker, self).__init__()
        self.title("BlueROV2 Position Tracker")
        self.geometry("1280x900")
        self.minsize(1050, 760)
        self.events = queue.Queue()
        self.rovl_worker = None
        self.gps_worker = None
        self.scan_worker = None
        self.rovl_message = None
        self.relative = None
        self.gps = {"fix": False, "lat": None, "lon": None}
        self.trail = deque(maxlen=500)
        self.arrivals = deque(maxlen=30)
        self.last_rovl_time = None
        self.records = []
        self.record_file = None
        self.recording = False
        self.rovl_var = tk.StringVar()
        self.gps_var = tk.StringVar()
        self.gps_baud_var = tk.IntVar(value=9600)
        self.status_var = tk.StringVar(value="Pronto — sola lettura")
        self.last_update_var = tk.StringVar(value="Last update: —")
        self.rate_var = tk.StringVar(value="Update rate: —")
        self.relative_values = {name: tk.StringVar(value="—") for name in ("slant", "horizontal", "bearing", "elevation", "east", "north")}
        self.topside_var = tk.StringVar(value="Topside: —")
        self.rov_global_var = tk.StringVar(value="ROV: —")
        self.global_info_var = tk.StringVar(value="WGS84: — | Distanza: — | Bearing: —")
        self.relative_mode_var = tk.StringVar(value="Mode: —")
        self.rovl_badge = None
        self.gps_badge = None
        self.relative_canvas = None
        self.global_canvas = None
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(50, self._poll_events)
        self.after(250, self._update_age)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")
        self.rovl_badge = self._status_badge(top, "ROVL Mk III", 0, "OFFLINE")
        self.gps_badge = self._status_badge(top, "GPS Topside", 1, "NO FIX")
        ttk.Label(top, textvariable=self.last_update_var).grid(row=0, column=2, sticky="ew", padx=8)
        ttk.Label(top, textvariable=self.rate_var).grid(row=0, column=3, sticky="ew", padx=8)
        for column in range(4):
            top.columnconfigure(column, weight=1)

        controls = ttk.Frame(self, padding=(8, 0, 8, 8))
        controls.pack(fill="x")
        ttk.Button(controls, text="Connect", command=self.connect).pack(side="left", padx=2)
        ttk.Button(controls, text="Disconnect", command=self.disconnect).pack(side="left", padx=2)
        ttk.Button(controls, text="Scan COM", command=self.scan_com).pack(side="left", padx=2)
        ttk.Button(controls, text="Clear trail", command=self.clear_trail).pack(side="left", padx=2)
        ttk.Button(controls, text="Start recording", command=self.start_recording).pack(side="left", padx=2)
        ttk.Button(controls, text="Stop recording", command=self.stop_recording).pack(side="left", padx=2)
        ttk.Button(controls, text="Export CSV", command=self.export_csv).pack(side="left", padx=2)
        ttk.Label(controls, textvariable=self.status_var).pack(side="right", padx=4)

        ports = ttk.LabelFrame(self, text="Serial ports (read-only selection)", padding=6)
        ports.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Label(ports, text="ROVL COM").grid(row=0, column=0, sticky="w")
        self.rovl_combo = ttk.Combobox(ports, textvariable=self.rovl_var, width=14, state="normal")
        self.rovl_combo.grid(row=0, column=1, padx=(4, 15))
        ttk.Label(ports, text="GPS COM").grid(row=0, column=2, sticky="w")
        self.gps_combo = ttk.Combobox(ports, textvariable=self.gps_var, width=14, state="normal")
        self.gps_combo.grid(row=0, column=3, padx=(4, 15))
        ttk.Label(ports, text="GPS baud").grid(row=0, column=4, sticky="w")
        ttk.Entry(ports, textvariable=self.gps_baud_var, width=9).grid(row=0, column=5, padx=(4, 15))
        self.scan_detail_var = tk.StringVar(value="Nessuna scansione eseguita")
        ttk.Label(ports, textvariable=self.scan_detail_var).grid(row=0, column=6, sticky="w")
        ports.columnconfigure(6, weight=1)

        notebook = ttk.Notebook(self)
        notebook.pack(fill="both", expand=True, padx=8, pady=(0, 8))
        relative_tab = ttk.Frame(notebook, padding=8)
        global_tab = ttk.Frame(notebook, padding=8)
        notebook.add(relative_tab, text="RELATIVE POSITION")
        notebook.add(global_tab, text="GLOBAL POSITION")
        self._build_relative_tab(relative_tab)
        self._build_global_tab(global_tab)

    def _status_badge(self, parent, title, column, initial):
        frame = ttk.LabelFrame(parent, text=title, padding=5)
        frame.grid(row=0, column=column, sticky="ew", padx=3)
        state = tk.StringVar(value=initial)
        label = ttk.Label(frame, textvariable=state, foreground="#b00020")
        label.pack()
        return {"state": state, "label": label}

    def _build_relative_tab(self, parent):
        body = ttk.Frame(parent)
        body.pack(fill="both", expand=True)
        self.relative_canvas = tk.Canvas(body, background="#101820", highlightthickness=0, width=self.CANVAS_SIZE, height=self.CANVAS_SIZE)
        self.relative_canvas.pack(side="left", fill="both", expand=True)
        values = ttk.LabelFrame(body, text="Relative position", padding=10)
        values.pack(side="right", fill="y", padx=(10, 0))
        labels = [("Slant range", "slant", "m"), ("Horizontal range", "horizontal", "m"), ("Bearing", "bearing", "deg"), ("Elevation", "elevation", "deg"), ("Relative East", "east", "m"), ("Relative North", "north", "m")]
        for row, (title, key, unit) in enumerate(labels):
            ttk.Label(values, text=title).grid(row=row, column=0, sticky="w", pady=5)
            ttk.Label(values, textvariable=self.relative_values[key], font=("Segoe UI", 12, "bold")).grid(row=row, column=1, sticky="e", pady=5)
            ttk.Label(values, text=unit).grid(row=row, column=2, sticky="w", pady=5)
        ttk.Label(values, textvariable=self.relative_mode_var, foreground="#9b4d00", wraplength=230, justify="left").grid(row=6, column=0, columnspan=3, pady=(15, 5), sticky="w")
        ttk.Label(values, text="N ↑   E →\nTopside receiver = (0, 0)", justify="left").grid(row=7, column=0, columnspan=3, pady=(10, 0), sticky="w")

    def _build_global_tab(self, parent):
        ttk.Label(parent, text="GLOBAL POSITION — local East/North diagnostic view; WGS84 lat/lon shown when TRUE bearing is available. No map tiles required.", foreground="#555555", wraplength=900).pack(anchor="w")
        body = ttk.Frame(parent)
        body.pack(fill="both", expand=True, pady=(5, 0))
        self.global_canvas = tk.Canvas(body, background="#102018", highlightthickness=0, width=self.CANVAS_SIZE, height=self.CANVAS_SIZE)
        self.global_canvas.pack(side="left", fill="both", expand=True)
        info = ttk.LabelFrame(body, text="Global coordinates", padding=10)
        info.pack(side="right", fill="y", padx=(10, 0))
        ttk.Label(info, textvariable=self.topside_var, justify="left").pack(anchor="w", pady=5)
        ttk.Label(info, textvariable=self.rov_global_var, justify="left").pack(anchor="w", pady=5)
        ttk.Label(info, textvariable=self.global_info_var, justify="left").pack(anchor="w", pady=5)

    def _set_badge(self, badge, text, online):
        badge["state"].set(text)
        badge["label"].configure(foreground="#087f23" if online else "#b00020")

    def scan_com(self):
        if self.scan_worker is not None and self.scan_worker.is_alive():
            return
        if serial is None:
            messagebox.showerror("pyserial", "Installa requirements_rovl.txt prima della scansione.")
            return
        self.status_var.set("Scansione COM in corso — nessun comando viene inviato")
        self.scan_worker = PortScanWorker(self.events)
        self.scan_worker.start()

    def connect(self):
        if serial is None:
            messagebox.showerror("pyserial", "Installa requirements_rovl.txt prima della connessione.")
            return
        if self.rovl_worker is None:
            port = self.rovl_var.get().strip()
            if port:
                self.rovl_worker = ROVLSerialWorker(port, self.events)
                self.rovl_worker.start()
            else:
                messagebox.showwarning("ROVL", "Seleziona o autodetecta una porta ROVL.")
        gps_port = self.gps_var.get().strip()
        rovl_port = self.rovl_var.get().strip()
        # A detected ROVL + GPS-retweet port already emits both event types;
        # opening the same COM a second time would be unsafe and unnecessary.
        same_rovl_port = bool(gps_port and rovl_port and gps_port.upper() == rovl_port.upper())
        if self.gps_worker is None and gps_port and not same_rovl_port:
            try:
                baud = int(self.gps_baud_var.get())
            except (TypeError, ValueError):
                messagebox.showerror("GPS", "Baud GPS non valido.")
                return
            self.gps_worker = GPSSerialWorker(gps_port, baud, self.events)
            self.gps_worker.start()
        elif same_rovl_port:
            self.status_var.set("ROVL + GPS retweet sulla stessa COM: un solo collegamento read-only")

    def disconnect(self):
        if self.rovl_worker is not None:
            self.rovl_worker.stop()
        if self.gps_worker is not None:
            self.gps_worker.stop()
        self.status_var.set("Disconnessione richiesta")

    def clear_trail(self):
        self.trail.clear()
        self._draw_views()

    def _poll_events(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == "scan_done":
                    self.rovl_combo["values"] = data["rovl"]
                    self.gps_combo["values"] = data["gps"]
                    if data["rovl"] and not self.rovl_var.get():
                        self.rovl_var.set(data["rovl"][0])
                    if data["gps"] and not self.gps_var.get():
                        self.gps_var.set(data["gps"][0])
                        self.gps_baud_var.set(data["gps_bauds"].get(data["gps"][0], 9600))
                    self.scan_detail_var.set("; ".join(data["details"]))
                    self.status_var.set("Scansione COM completata")
                elif kind == "rovl_connected":
                    self._set_badge(self.rovl_badge, "ONLINE", True)
                    self.status_var.set("ROVL connesso in sola lettura")
                elif kind == "rovl_message":
                    self._handle_rovl(data)
                elif kind == "rovl_warning":
                    self.status_var.set("ROVL: %s" % data)
                elif kind == "rovl_error":
                    self._set_badge(self.rovl_badge, "OFFLINE", False)
                    self.status_var.set("ROVL: %s" % data)
                elif kind == "rovl_closed":
                    self.rovl_worker = None
                    self._set_badge(self.rovl_badge, "OFFLINE", False)
                elif kind == "gps_connected":
                    self._set_badge(self.gps_badge, "NO FIX", False)
                elif kind == "gps_message":
                    self.gps = data
                    self._set_badge(self.gps_badge, "FIX" if data.get("fix") else "NO FIX", bool(data.get("fix")))
                elif kind == "gps_error":
                    self._set_badge(self.gps_badge, "NO FIX", False)
                    self.status_var.set("GPS: %s" % data)
                elif kind == "gps_closed":
                    self.gps_worker = None
                    self._set_badge(self.gps_badge, "NO FIX", False)
        except queue.Empty:
            pass
        self.after(50, self._poll_events)

    def _handle_rovl(self, message):
        self.rovl_message = message
        self.relative = relative_position(message)
        now = time.time()
        self.last_rovl_time = now
        self.arrivals.append(now)
        if self.relative is not None:
            geodetic = None
            if self.relative.get("global_eligible"):
                geodetic = forward_geodesic(self.gps.get("lat"), self.gps.get("lon"), self.relative["bearing_deg"], self.relative["horizontal_range_m"])
            entry = {"timestamp": datetime.now().isoformat(timespec="milliseconds"), "message": message, "relative": self.relative, "rov_lat": geodetic[0] if geodetic else None, "rov_lon": geodetic[1] if geodetic else None, "topside_lat": self.gps.get("lat"), "topside_lon": self.gps.get("lon")}
            self.trail.append(entry)
            self._update_values()
            self._record(entry)
        self._draw_views()

    def _update_values(self):
        if self.relative is None:
            return
        mapping = {"slant": self.relative["slant_range_m"], "horizontal": self.relative["horizontal_range_m"], "bearing": self.relative["bearing_deg"], "elevation": self.relative["elevation_deg"], "east": self.relative["east_m"], "north": self.relative["north_m"]}
        for key, value in mapping.items():
            unit = "°" if key in ("bearing", "elevation") else "m"
            self.relative_values[key].set("%.2f %s" % (value, unit))
        self.relative_mode_var.set("Mode: %s" % self.relative.get("mode", "UNKNOWN"))
        if self.trail:
            latest = self.trail[-1]
            self.topside_var.set("Topside: %s, %s" % (self._coord_text(latest["topside_lat"]), self._coord_text(latest["topside_lon"])))
            self.rov_global_var.set("ROV: %s, %s" % (self._coord_text(latest["rov_lat"]), self._coord_text(latest["rov_lon"])))
            global_mode = "TRUE bearing" if self.relative.get("global_eligible") else "unavailable (APPARENT / UNCOMPENSATED)"
            self.global_info_var.set("WGS84: %s | Distanza: %.2f m | Bearing: %.2f°" % (global_mode, self.relative["horizontal_range_m"], self.relative["bearing_deg"]))

    @staticmethod
    def _coord_text(value):
        return "—" if value is None else "%.7f°" % value

    def _update_age(self):
        if self.last_rovl_time is None:
            self.last_update_var.set("Last update: —")
            self.rate_var.set("Update rate: —")
        else:
            age = max(0.0, time.time() - self.last_rovl_time)
            self.last_update_var.set("Last update: %.1f s" % age)
            if len(self.arrivals) >= 2:
                rate = (len(self.arrivals) - 1) / max(1e-6, self.arrivals[-1] - self.arrivals[0])
                self.rate_var.set("Update rate: %.2f Hz" % rate)
        self.after(250, self._update_age)

    def _draw_views(self):
        self._draw_canvas(self.relative_canvas, "relative")
        self._draw_canvas(self.global_canvas, "global")

    def _draw_canvas(self, canvas, mode):
        if canvas is None:
            return
        canvas.delete("all")
        width = max(400, canvas.winfo_width())
        height = max(400, canvas.winfo_height())
        cx, cy = width / 2.0, height / 2.0
        points = [entry["relative"] for entry in self.trail if entry.get("relative")]
        max_range = max([10.0] + [abs(p["east_m"]) for p in points] + [abs(p["north_m"]) for p in points])
        scale = min(width, height) * 0.40 / (max_range * 1.25)
        radius = max_range * 1.25
        for step in self._grid_steps(radius):
            pixels = step * scale
            canvas.create_oval(cx - pixels, cy - pixels, cx + pixels, cy + pixels, outline="#42616b")
            canvas.create_text(cx + 4, cy - pixels - 8, text="%g m" % step, fill="#aac4ca", anchor="w")
        canvas.create_line(cx, 10, cx, height - 10, fill="#617d85")
        canvas.create_line(10, cy, width - 10, cy, fill="#617d85")
        canvas.create_text(cx + 8, 12, text="N", fill="white", anchor="w")
        canvas.create_text(width - 12, cy - 8, text="E", fill="white", anchor="e")
        trail_points = []
        for point in points:
            trail_points.append((cx + point["east_m"] * scale, cy - point["north_m"] * scale))
        if len(trail_points) > 1:
            canvas.create_line(trail_points, fill="#45a9ff", width=2)
        canvas.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#ffffff", outline="")
        canvas.create_text(cx + 9, cy + 9, text="TOPSIDE", fill="white", anchor="nw")
        if self.relative is not None:
            x = cx + self.relative["east_m"] * scale
            y = cy - self.relative["north_m"] * scale
            canvas.create_line(cx, cy, x, y, fill="#ffcc33", width=2, arrow=tk.LAST)
            canvas.create_oval(x - 6, y - 6, x + 6, y + 6, fill="#ff4f64", outline="white")
            canvas.create_text(x + 9, y, text="ROV", fill="white", anchor="w")
        if mode == "global":
            canvas.create_text(12, height - 15, text="WGS84 forward geodesic; local fallback view", fill="#b5cfc0", anchor="w")

    @staticmethod
    def _grid_steps(radius):
        target = radius / 4.0
        step = 1.0
        while step < target:
            step *= 2.0 if step < 5 else 2.5
        return [step * index for index in range(1, 6) if step * index <= radius]

    def _record(self, entry):
        message = entry["message"]
        relative = entry["relative"]
        record = {
            "timestamp": entry["timestamp"],
            "raw": message["raw"],
            "topside_lat": entry["topside_lat"],
            "topside_lon": entry["topside_lon"],
            "rov_lat": entry["rov_lat"],
            "rov_lon": entry["rov_lon"],
            "slant_range_m": relative["slant_range_m"],
            "horizontal_range_m": relative["horizontal_range_m"],
            "bearing_deg": relative["bearing_deg"],
            "elevation_deg": relative["elevation_deg"],
            "east_m": relative["east_m"],
            "north_m": relative["north_m"],
        }
        self.records.append(record)
        if self.record_file is not None:
            self.record_file.write(json.dumps(record, ensure_ascii=False) + "\n")
            self.record_file.flush()

    def start_recording(self):
        if self.record_file is not None:
            return
        directory = RECORD_ROOT / time.strftime("%Y%m%d_%H%M%S")
        directory.mkdir(parents=True, exist_ok=True)
        self.record_file = open(str(directory / "rovl_positions.jsonl"), "w", encoding="utf-8")
        self.record_file.write(json.dumps({"metadata": {"started": datetime.now().isoformat(), "baud": ROVL_BAUD}}) + "\n")
        self.record_file.flush()
        self.recording = True
        self.status_var.set("Registrazione ROVL: %s" % directory)

    def stop_recording(self):
        if self.record_file is not None:
            self.record_file.close()
        self.record_file = None
        self.recording = False
        self.status_var.set("Registrazione ROVL fermata")

    def export_csv(self):
        if not self.records:
            messagebox.showinfo("Export CSV", "Nessuna posizione registrata.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")], initialfile="rovl_positions_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8") as output:
            writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(self.records)
        self.status_var.set("CSV esportato: %s" % os.path.basename(path))

    def save_screenshot(self):
        path = filedialog.asksaveasfilename(defaultextension=".png", filetypes=[("PNG", "*.png")], initialfile="rovl_tracker_%s.png" % time.strftime("%Y%m%d_%H%M%S"))
        if not path:
            return
        try:
            x = self.winfo_rootx()
            y = self.winfo_rooty()
            ImageGrab.grab(bbox=(x, y, x + self.winfo_width(), y + self.winfo_height())).save(path)
            self.status_var.set("Screenshot salvato: %s" % os.path.basename(path))
        except Exception as exc:
            messagebox.showerror("Screenshot", str(exc))

    def close(self):
        self.stop_recording()
        self.disconnect()
        if self.rovl_worker is not None:
            self.rovl_worker.join(timeout=1.0)
        if self.gps_worker is not None:
            self.gps_worker.join(timeout=1.0)
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description="BlueROV2 Position Tracker")
    parser.add_argument("--offline", action="store_true", help="open GUI without serial connections")
    args = parser.parse_args()
    app = PositionTracker()
    if not args.offline:
        pass
    app.mainloop()


if __name__ == "__main__":
    main()
