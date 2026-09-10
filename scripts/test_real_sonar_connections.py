"""Read-only diagnostics for the real BlueROV2 sonar network paths.

Checks:
  1. BlueOS reachability on HTTP and/or mavlink2rest port 6040.
  2. TCP connect/close to Omniscan 450 at 192.168.2.86:51200.
  3. Ping1D UDP PingProxy initialization at 192.168.2.2:9090.

This script never sends an Omniscan os_ping_params enable=1 command.
It does not change BlueOS, sonar, serial, or network configuration.
"""

from __future__ import print_function

import argparse
import socket
import sys
import time


DEFAULT_BLUEOS = "192.168.2.2"
DEFAULT_OMNI = "192.168.2.86"
DEFAULT_OMNI_PORT = 51200
DEFAULT_PING_PORT = 9090


def tcp_probe(host, port, timeout):
    started = time.time()
    try:
        sock = socket.create_connection((host, port), timeout)
        sock.close()
        return True, (time.time() - started) * 1000.0, "TCP connect ok"
    except OSError as exc:
        return False, (time.time() - started) * 1000.0, str(exc)


def ping1d_probe(host, port):
    try:
        from brping import Ping1D
    except ImportError as exc:
        return False, "bluerobotics-ping non installato: %s" % exc
    device = None
    try:
        device = Ping1D()
        device.connect_udp(host, port)
        ok = bool(device.initialize())
        return ok, "initialize()=%s; nessun avvio Omniscan" % ok
    except Exception as exc:
        return False, str(exc)
    finally:
        if device is not None:
            io_device = getattr(device, "iodev", None)
            if io_device is not None:
                try:
                    io_device.close()
                except Exception:
                    pass


def main():
    parser = argparse.ArgumentParser(description="Read-only BlueROV2 sonar connection diagnostics")
    parser.add_argument("--blueos", default=DEFAULT_BLUEOS)
    parser.add_argument("--omniscan", default=DEFAULT_OMNI)
    parser.add_argument("--omniscan-port", type=int, default=DEFAULT_OMNI_PORT)
    parser.add_argument("--ping1d-port", type=int, default=DEFAULT_PING_PORT)
    args = parser.parse_args()

    print("BlueROV2 sonar connection diagnostics (read-only)")
    print("Do not run SonarView/PingViewer at the same time.")
    print()

    http_ok, http_ms, http_detail = tcp_probe(args.blueos, 80, 1.5)
    rest_ok, rest_ms, rest_detail = tcp_probe(args.blueos, 6040, 1.5)
    blueos_ok = http_ok or rest_ok
    print("BlueOS       %s  HTTP:%s (%.0f ms), mavlink2rest:%s (%.0f ms)" % (
        "ONLINE" if blueos_ok else "OFFLINE",
        "OK" if http_ok else "FAIL",
        http_ms,
        "OK" if rest_ok else "FAIL",
        rest_ms,
    ))
    if not http_ok:
        print("  HTTP detail: %s" % http_detail)
    if not rest_ok:
        print("  REST detail: %s" % rest_detail)

    omni_ok, omni_ms, omni_detail = tcp_probe(args.omniscan, args.omniscan_port, 2.0)
    print("Omniscan     %s  %s:%d (%.0f ms)" % (
        "ONLINE" if omni_ok else "OFFLINE", args.omniscan, args.omniscan_port, omni_ms
    ))
    print("  %s; no enable=1 command sent" % omni_detail)

    ping_ok, ping_detail = ping1d_probe(args.blueos, args.ping1d_port)
    print("Ping1D       %s  %s:%d" % (
        "ONLINE" if ping_ok else "OFFLINE", args.blueos, args.ping1d_port
    ))
    print("  %s" % ping_detail)

    return 0 if blueos_ok and omni_ok and ping_ok else 1


if __name__ == "__main__":
    sys.exit(main())
