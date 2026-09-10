"""Read-only COM diagnostics for a Cerulean ROV Locator Mk III.

The script lists COM ports, reads each candidate without writing, identifies
valid $USRTH ROVL output and common NMEA GPS output, and prints the last
message observed.  No ROVL configuration or command is transmitted.
"""

from __future__ import print_function

import argparse
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parents[0] / "real"
sys.path.insert(0, str(SCRIPT_DIR))

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    serial = None
    list_ports = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

from rovl_protocol import classify_serial_sentence, parse_gps_sentence  # noqa: E402


ROVL_BAUD = 115200
GPS_BAUDS = (4800, 9600, 38400, 115200)


def read_probe(port, baud, seconds, parser):
    if serial is None:
        return None
    device = None
    last = None
    try:
        device = serial.Serial(port, baudrate=baud, timeout=0.15)
        deadline = time.time() + seconds
        while time.time() < deadline:
            line = device.readline()
            if not line:
                continue
            try:
                parsed = parser(line)
            except ValueError:
                continue
            if parsed is not None:
                last = parsed
        return last
    except Exception:
        return None
    finally:
        if device is not None:
            try:
                device.close()
            except Exception:
                pass


def read_combined_probe(port, baud, seconds):
    """Read one ROVL port and retain both $USRTH and GPS retweets."""
    result = {"rovl": None, "gps": None}
    if serial is None:
        return result
    device = None
    try:
        device = serial.Serial(port, baudrate=baud, timeout=0.15)
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


def main():
    parser = argparse.ArgumentParser(description="Read-only ROVL/GPS COM diagnostics")
    parser.add_argument("--seconds", type=float, default=1.5, help="read time per baud/port")
    args = parser.parse_args()
    if serial is None:
        print("pyserial non installato: %s" % SERIAL_IMPORT_ERROR)
        return 2
    ports = [item.device for item in list_ports.comports()]
    print("ROVL Mk III / GPS diagnostics — READ ONLY")
    print("COM ports: %s" % (", ".join(ports) if ports else "nessuna"))
    rovl_found = []
    gps_found = []
    for port in ports:
        combined = read_combined_probe(port, ROVL_BAUD, args.seconds)
        rovl = combined["rovl"]
        retweeted_gps = combined["gps"]
        if rovl is not None:
            rovl_found.append(port)
            print("ROVL %s @ %d: $USRTH OK; last raw: %s" % (port, ROVL_BAUD, rovl["raw"]))
        if retweeted_gps is not None:
            gps_found.append((port, ROVL_BAUD))
            print("GPS  %s @ %d: %s retweet; last raw: %s" % (port, ROVL_BAUD, "FIX" if retweeted_gps.get("fix") else "NO FIX", retweeted_gps["raw"]))
        if rovl is not None and retweeted_gps is not None:
            print("COM %s identificata come ROVL + GPS retweet" % port)
        if retweeted_gps is not None:
            continue
        for baud in GPS_BAUDS:
            gps = read_probe(port, baud, args.seconds, parse_gps_sentence)
            if gps is not None:
                gps_found.append((port, baud))
                print("GPS  %s @ %d: %s; last raw: %s" % (port, baud, "FIX" if gps.get("fix") else "NO FIX", gps["raw"]))
                break
    if not rovl_found:
        print("ROVL: nessun $USRTH valido rilevato")
    if not gps_found:
        print("GPS: nessun NMEA GGA/RMC valido rilevato")
    print("Comandi inviati: nessuno")
    return 0


if __name__ == "__main__":
    sys.exit(main())
