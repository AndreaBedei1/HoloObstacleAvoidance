# Real Pool Setup — Survey and Pre-Run Configuration (2026-08-13)

Overhead RealSense D435 mounted above the pool; ROV NOT connected (all
safety rules active: no arming, no thrusters, interlocks default-false,
imaging sonar prohibited). Everything in this document is read-only
measurement + configuration; no simulation and no vehicle interaction.

## Scene

~6 m pool with a transverse rod at 3.92 m from the left end; the anchor
hangs from the rod middle (suspension point 7 cm toward the far rim) and
nearly touches the bottom where the water is ~1.3 m deep. Left segment
(the longer one) starts at ~0.6 m depth; the far right shelf reaches
~0.8 m. The ROV must pass the anchor laterally (left or right).

Artifacts: `visualizations/pool_setup_20260812/` (RGB 1080p, aligned
median depth, annotated points, anchor-zone zoom, inventory JSON,
`pool_geometry.json`, `pool_frame.json`, `pool_depth_crosscheck.json`).

## Verification of the operator's measures

| Quantity | Operator | Camera (method) | Verdict |
|---|---|---|---|
| Pool length | ~6 m | 5.59 m visible; right end just past frame edge | ✔ consistent |
| Rod position | "center-ish, left part longer" | 3.92 m from left end (65/35 split) | ✔ |
| Pool width | — | 2.66 m at rod, 3.05 m left | measured |
| Left-zone depth | 0.60 m | 0.567 m (refraction-corrected stereo, ×1.333 vs surface-plane fit, rim residual 1.3 cm) | ✔ (3 cm) |
| Anchor-zone depth | 1.30 m | 1.221 m | ✔ (8 cm; IR attenuation) |
| Right-zone depth | 0.80 m | 1.29 m in the VISIBLE right zone | ⚠ the 0.80 m shelf lies at/just beyond the right frame edge — confirm its x position |
| Anchor suspension | rod middle | pixel (1268, 540) → pool (3.92, +0.07) m | ✔ |

Camera: ~3.7 m above the surface, tilted; camera→pool rigid transform and
surface plane recorded in `config/real_pool/pool_geometry.yaml` (GT uses
ABOVE-water returns only; underwater stereo is refraction-biased and used
only for the ×1.333-corrected bottom cross-check).

## Operational envelope (from measured geometry)

- **Vertical:** the left shelf (0.57–0.8 m to x≈2 m) cannot host the
  vehicle at maneuvering depth (keel clearance < 0.1 m) — start the run at
  x ≥ 2.3 m in the 1.2–1.3 m basin; proposed mission depth 0.55 m
  (keel 0.75, ~0.5 m bottom clearance); stop by x ≈ 5.5 m until the right
  shelf is confirmed.
- **Lateral:** usable clearance anchor-center → near rim ≈ 1.40 m;
  after 0.25 m wall standoff and the 0.40 m footprint, the max real margin
  is ~0.75 m minus the anchor half-width. The Phase-8 sim margin (0.80 m)
  DOES NOT FIT — pool profile proposes 0.35 m (D-014 draft in
  `config/real_pool/pool_mission_profile.yaml`).
- **Route:** usable 3.2 m (x 2.3→5.5), anchor at 3.92 m, 1.6 m of
  detection run-up at 0.12 m/s (adequate for T2 warm-up + confirmation).

## Corridor analysis with the 0.80 × 0.80 m anchor (operator estimate)

Free water beside the anchor (near-rim side): 1.399 − 0.40 ≈ **1.0 m**.
The sim-circumscribed vehicle radius (0.40 m) leaves NO admissible pass
for any margin ≥ 0.1 m — the pool profile therefore plans with the
orientation-aware transverse half-width (0.457/2 + 0.05 = **0.28 m**,
valid because the DWA holds yaw ≈ 0 in sway passes) and margin 0.15 m:
hull-center window [0.83, 1.02] m from the anchor center, pass at
~0.92 m, physical clearances ≈ 0.29 m (anchor side) / 0.25 m (wall side)
at 0.12 m/s. **Recommendation:** if practical, shift the suspension
0.3–0.4 m toward the far rim — the corridor grows to ~1.4 m and every
margin doubles.

## Open items before any wet run (Andrea)

1. ~~Measure the anchor~~ **DONE 2026-08-13: ~0.80 m wide × 0.80 m tall
   (operator estimate)** — to be refined tomorrow with the drone camera;
   H_ref 0.80 configured for BOTH planners.
2. **Confirm the right shelf**: where the bottom rises to 0.80 m
   (camera sees ≥1.2 m everywhere in-frame on the right).
3. **Committed planner engagement re-scale** (Phase-8 K1 finding: it never
   engaged the small pool class and grazed at 0.10 m) — values after (1).
4. Camera mount rigidity check (any bump invalidates the camera→pool
   transform; re-run `scripts/measure_pool_geometry.py` after any touch).
5. Decide anchor suspension height if the visual engagement geometry at
   0.55 m depth proves poor (anchor top vs camera axis).

Safety unchanged: interlocks default-false, arming operator-only, Ping-1D
only after in-water confirmation, imaging sonar prohibited forever.
