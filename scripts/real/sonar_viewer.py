"""Small read-only live viewer for a Cerulean Omniscan 450.

The viewer speaks the Cerulean/Ping Protocol directly over TCP.  The
Omniscan 450 uses TCP port 51200 and returns one ``os_mono_profile`` packet
per ping.  Each profile is drawn as one horizontal row in a scrolling
waterfall image: x = range, y = time (newest row at the bottom).

The application does not touch MAVLink, the BlueOS APIs, or the ROV
actuators.  The only command sent to the sonar is the explicit Start/Stop
Ping button.  Close SonarView before connecting if it already owns the sonar.

Reference:
https://docs.ceruleansonar.com/c/omniscan-450/application-programming-interface
https://docs.ceruleansonar.com/c/sonarview/mission-configurations/omniscan-450
"""

from __future__ import print_function

import argparse
import io
import os
import queue
import socket
import struct
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import filedialog, messagebox, ttk

from PIL import Image, ImageTk


PACKET_START = b"BR"
PACKET_HEADER = struct.Struct("<BBHHBB")
PACKET_CHECKSUM = struct.Struct("<H")
OS_PING_PARAMS = 2197
OS_MONO_PROFILE = 2198

# os_ping_params payload:
# u32 start_mm, u32 length_mm, u32 msec_per_ping,
# float reserved, float reserved, float pulse_len_percent,
# float filter_duration_percent, i16 gain_index, u16 num_points,
# u8 enable, u8 reserved x3.
PING_PARAMS = struct.Struct("<IIIffffhH4B")

# Fixed part of os_mono_profile before pwr_results[].
PROFILE_HEADER = struct.Struct("<IIIIIHHHBBffffff")


def make_packet(packet_id, payload):
    """Build one little-endian Cerulean packet with its additive checksum."""
    header = PACKET_HEADER.pack(66, 82, len(payload), packet_id, 0, 0)
    checksum = sum(header + payload) & 0xFFFF
    return header + payload + PACKET_CHECKSUM.pack(checksum)


def ping_params(length_mm, points, gain_index, enable, ping_hz=8.0):
    """Build a conservative Omniscan start/stop command."""
    # 0 means best available rate according to the vendor API.  For a live
    # viewer we use a bounded rate so the first lake test is not aggressive.
    msec_per_ping = int(round(1000.0 / ping_hz)) if ping_hz > 0 else 0
    return PING_PARAMS.pack(
        0,
        int(length_mm),
        msec_per_ping,
        0.0,
        0.0,
        0.002,
        0.0015,
        int(gain_index),
        int(points),
        1 if enable else 0,
        0,
        0,
        0,
    )


class PacketReader(object):
    """Incremental parser for the TCP byte stream."""

    def __init__(self, sock):
        self.sock = sock
        self.buffer = bytearray()

    def _fill(self, count):
        while len(self.buffer) < count:
            chunk = self.sock.recv(max(4096, count - len(self.buffer)))
            if not chunk:
                raise ConnectionError("sonar closed the TCP connection")
            self.buffer.extend(chunk)

    def next_packet(self):
        while True:
            self._fill(2)
            if self.buffer[:2] == PACKET_START:
                break
            del self.buffer[0]

        self._fill(8)
        _, _, payload_len, packet_id, src, dst = PACKET_HEADER.unpack(
            self.buffer[:8]
        )
        total = 8 + payload_len + 2
        self._fill(total)
        raw = bytes(self.buffer[:total])
        del self.buffer[:total]

        expected = PACKET_CHECKSUM.unpack(raw[-2:])[0]
        actual = sum(raw[:-2]) & 0xFFFF
        if expected != actual:
            raise ValueError(
                "bad sonar checksum: expected %d, got %d" % (expected, actual)
            )
        return packet_id, raw[8:-2]


def decode_profile(payload):
    """Return display metadata and a normalized 8-bit power row."""
    if len(payload) < PROFILE_HEADER.size:
        raise ValueError("short os_mono_profile payload")

    values = PROFILE_HEADER.unpack_from(payload)
    (
        ping_number,
        start_mm,
        length_mm,
        timestamp_ms,
        ping_hz,
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

    available = (len(payload) - PROFILE_HEADER.size) // 2
    count = min(int(num_results), available)
    if count <= 0:
        return None

    raw_values = struct.unpack_from("<%dH" % count, payload, PROFILE_HEADER.size)
    lo = float(min_pwr_db)
    hi = float(max_pwr_db)
    if hi <= lo:
        row = [0] * count
    else:
        row = [
            max(0, min(255, int(round((value / 65535.0) * 255.0))))
            for value in raw_values
        ]

    return {
        "ping_number": ping_number,
        "start_mm": start_mm,
        "length_mm": length_mm,
        "timestamp_ms": timestamp_ms,
        "ping_hz": ping_hz,
        "gain_index": gain_index,
        "num_results": count,
        "sos_dmps": sos_dmps,
        "channel_number": channel_number,
        "pulse_duration_sec": pulse_duration_sec,
        "analog_gain": analog_gain,
        "min_pwr_db": min_pwr_db,
        "max_pwr_db": max_pwr_db,
        "transducer_heading_deg": transducer_heading_deg,
        "vehicle_heading_deg": vehicle_heading_deg,
        "row": row,
    }


def colorize(row):
    """Blue-to-yellow sonar palette, returned as RGB bytes."""
    out = bytearray(len(row) * 3)
    for i, value in enumerate(row):
        x = value / 255.0
        # Keep weak returns dark blue and strong returns yellow/white.
        r = int(255 * max(0.0, min(1.0, (x - 0.42) * 2.2)))
        g = int(255 * max(0.0, min(1.0, (x - 0.18) * 1.55)))
        b = int(255 * max(0.0, min(1.0, 0.25 + x * 0.9)))
        j = i * 3
        out[j : j + 3] = bytes((r, g, b))
    return bytes(out)


class SonarWorker(threading.Thread):
    def __init__(self, host, port, events):
        super(SonarWorker, self).__init__(daemon=True)
        self.host = host
        self.port = port
        self.events = events
        self.stop_event = threading.Event()
        self.sock = None
        self.reader = None
        self.pinging = False

    def send_ping_command(self, enable, length_mm, points, gain, ping_hz):
        if self.sock is None:
            raise RuntimeError("not connected")
        payload = ping_params(length_mm, points, gain, enable, ping_hz)
        self.sock.sendall(make_packet(OS_PING_PARAMS, payload))
        self.pinging = bool(enable)

    def run(self):
        try:
            self.sock = socket.create_connection((self.host, self.port), 5.0)
            self.sock.settimeout(1.0)
            self.reader = PacketReader(self.sock)
            self.events.put(("connected", None))
            while not self.stop_event.is_set():
                try:
                    packet_id, payload = self.reader.next_packet()
                except socket.timeout:
                    continue
                if packet_id != OS_MONO_PROFILE:
                    continue
                try:
                    profile = decode_profile(payload)
                except (ValueError, struct.error) as exc:
                    self.events.put(("warning", str(exc)))
                    continue
                if profile is not None:
                    self.events.put(("profile", profile))
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            if self.sock is not None:
                try:
                    if self.pinging:
                        self.sock.sendall(
                            make_packet(
                                OS_PING_PARAMS,
                                ping_params(5000, 600, -1, False),
                            )
                        )
                except Exception:
                    pass
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.events.put(("closed", None))

    def stop(self):
        self.stop_event.set()
        if self.sock is not None:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self.sock.close()
            except Exception:
                pass


class SonarViewer(tk.Tk):
    WIDTH = 760
    HEIGHT = 420

    def __init__(self, host, port):
        super(SonarViewer, self).__init__()
        self.title("Omniscan 450 — live sonar viewer")
        self.geometry("940x650")
        self.minsize(820, 560)
        self.host = tk.StringVar(value=host)
        self.port = tk.IntVar(value=port)
        self.range_m = tk.DoubleVar(value=20.0)
        self.points = tk.IntVar(value=600)
        self.gain = tk.IntVar(value=-1)
        self.ping_hz = tk.DoubleVar(value=8.0)
        self.status = tk.StringVar(value="Disconnesso")
        self.info = tk.StringVar(value="Nessun profilo ricevuto")
        self.events = queue.Queue()
        self.worker = None
        self.running = False
        self.rows = deque(maxlen=self.HEIGHT)
        self.latest_image = None
        self.photo = None
        self._build_ui()
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(50, self._poll_events)

    def _build_ui(self):
        controls = ttk.Frame(self, padding=8)
        controls.pack(fill="x")
        ttk.Label(controls, text="Sonar IP").grid(row=0, column=0, sticky="w")
        ttk.Entry(controls, textvariable=self.host, width=15).grid(
            row=0, column=1, padx=(4, 12)
        )
        ttk.Label(controls, text="Porta").grid(row=0, column=2, sticky="w")
        ttk.Entry(controls, textvariable=self.port, width=7).grid(
            row=0, column=3, padx=(4, 12)
        )
        self.connect_button = ttk.Button(
            controls, text="Connetti", command=self.connect
        )
        self.connect_button.grid(row=0, column=4, padx=3)
        self.start_button = ttk.Button(
            controls, text="Avvia ping", command=self.start_ping, state="disabled"
        )
        self.start_button.grid(row=0, column=5, padx=3)
        self.stop_button = ttk.Button(
            controls, text="Ferma ping", command=self.stop_ping, state="disabled"
        )
        self.stop_button.grid(row=0, column=6, padx=3)
        self.save_button = ttk.Button(
            controls,
            text="Salva PNG",
            command=self.save_image,
            state="disabled",
        )
        self.save_button.grid(row=0, column=7, padx=3)

        settings = ttk.LabelFrame(self, text="Parametri ping", padding=8)
        settings.pack(fill="x", padx=8, pady=(0, 8))
        ttk.Label(settings, text="Portata (m)").grid(row=0, column=0, sticky="w")
        ttk.Entry(settings, textvariable=self.range_m, width=8).grid(
            row=0, column=1, padx=(4, 12)
        )
        ttk.Label(settings, text="Campioni").grid(row=0, column=2, sticky="w")
        ttk.Entry(settings, textvariable=self.points, width=8).grid(
            row=0, column=3, padx=(4, 12)
        )
        ttk.Label(settings, text="Gain (-1 auto)").grid(row=0, column=4, sticky="w")
        ttk.Entry(settings, textvariable=self.gain, width=8).grid(
            row=0, column=5, padx=(4, 12)
        )
        ttk.Label(settings, text="Ping/s").grid(row=0, column=6, sticky="w")
        ttk.Entry(settings, textvariable=self.ping_hz, width=8).grid(
            row=0, column=7, padx=(4, 12)
        )

        self.image_label = ttk.Label(self, text="Premi Connetti, poi Avvia ping")
        self.image_label.pack(fill="both", expand=True, padx=8, pady=8)
        ttk.Label(self, textvariable=self.info, anchor="w").pack(
            fill="x", padx=8, pady=(0, 2)
        )
        ttk.Label(self, textvariable=self.status, anchor="w").pack(
            fill="x", padx=8, pady=(0, 8)
        )

    def _validate_params(self):
        length_mm = int(round(float(self.range_m.get()) * 1000.0))
        points = int(self.points.get())
        gain = int(self.gain.get())
        ping_hz = float(self.ping_hz.get())
        if not 1.0 <= length_mm <= 150000.0:
            raise ValueError("La portata deve essere tra 0.001 e 150 m")
        if not 200 <= points <= 1200:
            raise ValueError("I campioni devono essere tra 200 e 1200")
        if not -1 <= gain <= 7:
            raise ValueError("Il gain deve essere -1 (auto) oppure 0..7")
        if not 0.2 <= ping_hz <= 20.0:
            raise ValueError("Ping/s deve essere tra 0.2 e 20")
        return length_mm, points, gain, ping_hz

    def connect(self):
        if self.worker is not None:
            return
        try:
            port = int(self.port.get())
        except (TypeError, ValueError):
            messagebox.showerror("Porta non valida", "Inserisci una porta TCP valida.")
            return
        self.status.set("Connessione a %s:%d..." % (self.host.get(), port))
        self.connect_button.configure(state="disabled")
        self.worker = SonarWorker(self.host.get().strip(), port, self.events)
        self.worker.start()

    def start_ping(self):
        if self.worker is None or self.worker.sock is None:
            return
        try:
            params = self._validate_params()
            self.worker.send_ping_command(True, *params)
            self.running = True
            self.start_button.configure(state="disabled")
            self.stop_button.configure(state="normal")
            self.status.set("Ping attivo — acquisizione in corso")
        except Exception as exc:
            messagebox.showerror("Avvio sonar fallito", str(exc))

    def stop_ping(self):
        if self.worker is None or self.worker.sock is None:
            return
        try:
            params = self._validate_params()
            self.worker.send_ping_command(False, *params)
        except Exception as exc:
            self.status.set("Stop non confermato: %s" % exc)
        self.running = False
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")
        self.status.set("Ping fermato")

    def _poll_events(self):
        try:
            while True:
                kind, data = self.events.get_nowait()
                if kind == "connected":
                    self.start_button.configure(state="normal")
                    self.status.set("Connesso a %s:%s — ping fermo" % (self.host.get(), self.port.get()))
                elif kind == "profile":
                    self._add_profile(data)
                elif kind == "warning":
                    self.status.set("Avviso: %s" % data)
                elif kind == "error":
                    self.status.set("Errore: %s" % data)
                    self.connect_button.configure(state="normal")
                elif kind == "closed":
                    self.worker = None
                    self.running = False
                    self.connect_button.configure(state="normal")
                    self.start_button.configure(state="disabled")
                    self.stop_button.configure(state="disabled")
                    if self.status.get().startswith("Ping attivo"):
                        self.status.set("Connessione chiusa")
        except queue.Empty:
            pass
        self.after(50, self._poll_events)

    def _add_profile(self, profile):
        width = max(1, int(profile["num_results"]))
        row = colorize(profile["row"])
        if width != self.WIDTH:
            image = Image.new("RGB", (self.WIDTH, 1), (0, 0, 0))
            line = Image.frombytes("RGB", (width, 1), row)
            line.thumbnail((self.WIDTH, 1))
            image.paste(line, (0, 0))
            row = image.tobytes()
        self.rows.append(row)
        while len(self.rows) < self.HEIGHT:
            self.rows.appendleft(bytes(self.WIDTH * 3))
        image = Image.frombytes("RGB", (self.WIDTH, self.HEIGHT), b"".join(self.rows))
        self.latest_image = image
        preview = image.resize((self.WIDTH, self.HEIGHT), Image.Resampling.NEAREST)
        self.photo = ImageTk.PhotoImage(preview)
        self.image_label.configure(image=self.photo, text="")
        self.save_button.configure(state="normal")
        self.info.set(
            "Ping %d | portata %.2f m | %d campioni | %.1f ping/s | gain %d"
            % (
                profile["ping_number"],
                profile["length_mm"] / 1000.0,
                profile["num_results"],
                profile["ping_hz"],
                profile["gain_index"],
            )
        )

    def save_image(self):
        if self.latest_image is None:
            return
        path = filedialog.asksaveasfilename(
            title="Salva immagine sonar",
            defaultextension=".png",
            filetypes=[("PNG", "*.png")],
            initialfile="sonar_%s.png" % time.strftime("%Y%m%d_%H%M%S"),
        )
        if path:
            self.latest_image.save(path)
            self.status.set("Immagine salvata: %s" % os.path.basename(path))

    def close(self):
        if self.worker is not None:
            self.worker.stop()
            self.worker.join(timeout=1.5)
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description="Live viewer for Cerulean Omniscan 450")
    parser.add_argument("--host", default="192.168.2.86", help="sonar IPv4 address")
    parser.add_argument("--port", type=int, default=51200, help="sonar TCP port")
    args = parser.parse_args()
    app = SonarViewer(args.host, args.port)
    app.mainloop()


if __name__ == "__main__":
    main()
