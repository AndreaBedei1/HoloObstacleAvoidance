"""Draw the start region for the next run on a live overhead frame.

The operator cannot place a tethered vehicle by pixel coordinates, and
should not have to estimate 1.9 m by eye across a pool. This marks the
nominal start and its pre-registered tolerance box on the picture the
ground truth actually sees, so placing the vehicle is a matter of
matching what is on screen.

The vehicle's CURRENT position is drawn too, with its offset from the
nominal, so a nudge can be judged rather than guessed.

Usage:
    python scripts/real/show_start_region.py            # next run
    python scripts/real/show_start_region.py --geometry K1
"""

from __future__ import annotations

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

from camera_stream import ensure_env  # noqa: E402

ensure_env()

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from overhead_track import OverheadTracker  # noqa: E402

OUT = os.path.join(_ROOT, "experiments", "real", "start_region.jpg")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--geometry", choices=["K0", "K1", "K1M"])
    args = ap.parse_args()

    import final_campaign as fc
    seq, cfg = fc.frozen_order()

    # K1M is the mirrored offset geometry added during the session; it
    # lives in its own file so the frozen order stays untouched.
    _extra = os.path.join(_ROOT, "config", "geometry_K1M_session.yaml")
    if os.path.exists(_extra):
        import yaml
        with open(_extra) as _f:
            cfg["start_poses"].update(
                yaml.safe_load(_f).get("start_poses", {}))
    geom = args.geometry
    label = ""
    if geom is None:
        nxt = fc.next_pending(seq, fc.completed_runs())
        if nxt is None:
            print("le 20 prove sono complete")
            return 0
        geom = nxt["geometry"]
        label = "prova %d/20: %s %s replica %d" % (
            nxt["index"], nxt["geometry"], nxt["planner"], nxt["run"])
    sp = cfg["start_poses"][geom]
    nom = np.array(sp["nominal_px"], float)
    ppm = cfg["start_poses"]["px_per_m"]
    anchor = np.array(cfg["start_poses"]["anchor_px"], float)
    tol_along = cfg["tolerance"]["along_m"] * ppm
    tol_lat = cfg["tolerance"]["lateral_m"] * ppm

    tr = OverheadTracker()
    img, det = None, None
    for _ in range(8):
        d = tr.detect()
        f = getattr(tr, "last_frame", None)
        if f is not None:
            img, det = f.copy(), d
        time.sleep(0.12)
    tr.close()
    if img is None:
        print("nessun fotogramma dalla telecamera dall'alto")
        return 2

    # VISIBILITY LIMIT. The vehicle blob is about 200 px wide, so its
    # centre must stay this far from the frame edge for the whole
    # vehicle to be seen. The pre-registered tolerance box extends past
    # that limit on the near side -- the nominal start is already only
    # 200 px from the edge, because it is the furthest a measurable
    # start fits in the frame at all. A pose can therefore be inside
    # tolerance and still be half out of view, which is a WORSE state
    # than being out of tolerance: the ground truth is degraded without
    # the pose check complaining.
    vis_min_x = 115
    cv2.line(img, (vis_min_x, 0), (vis_min_x, img.shape[0]),
             (0, 128, 255), 2)
    cv2.putText(img, "limite visibilita", (vis_min_x + 8, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 128, 255), 2)

    # tolerance box: along the approach is image x, lateral is image y,
    # clipped to the region where the vehicle is wholly visible
    p1 = (max(vis_min_x, int(nom[0] - tol_along)),
          int(nom[1] - tol_lat))
    p2 = (int(nom[0] + tol_along), int(nom[1] + tol_lat))
    cv2.rectangle(img, p1, p2, (0, 255, 0), 3)
    cv2.circle(img, (int(nom[0]), int(nom[1])), 12, (0, 255, 0), -1)
    cv2.putText(img, "METTI QUI (%s)" % geom, (p1[0], max(30, p1[1] - 14)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
    cv2.circle(img, (int(anchor[0]), int(anchor[1])), 18, (0, 0, 255), 3)
    cv2.putText(img, "ANCORA", (int(anchor[0]) + 24, int(anchor[1])),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3)
    cv2.arrowedLine(img, (int(nom[0]), int(nom[1])),
                    (int(anchor[0]) - 40, int(anchor[1])),
                    (0, 255, 255), 2, tipLength=0.03)

    msg = "fuori inquadratura"
    if det and det.get("found"):
        px = np.array(det["pixel"], float)
        off = (px - nom) / ppm
        whole = px[0] >= vis_min_x and not det.get("partial")
        inside = (abs(off[0]) <= cfg["tolerance"]["along_m"]
                  and abs(off[1]) <= cfg["tolerance"]["lateral_m"]
                  and whole)
        col = (0, 255, 0) if inside else (255, 0, 255)
        cv2.circle(img, (int(px[0]), int(px[1])), 14, col, -1)
        b = det.get("bbox")
        if b:
            cv2.rectangle(img, (b[0], b[1]), (b[0] + b[2], b[1] + b[3]),
                          col, 3)
        msg = ("ROV: %+.2f m avanti/indietro, %+.2f m di lato%s  -> %s"
               % (off[0], off[1],
                  "" if whole else "  [MEZZO FUORI INQUADRATURA]",
                  "PRONTO" if inside else "da spostare"))
        cv2.putText(img, msg, (30, img.shape[0] - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
    if label:
        cv2.putText(img, label, (30, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (255, 255, 255), 2)

    cv2.imwrite(OUT, img)
    print(label or geom)
    print(msg)
    print("distanza nominale dall'ancora: %.2f m"
          % sp["approach_distance_m"])
    print("->", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
