# Real Wet-Test Safety Checklist

Execute IN ORDER, only after Andrea explicitly declares the BlueROV2 in water.
No step may be skipped. Autonomy comes LAST. Each step gets a logged
timestamp + operator initials in the session log.

## Preconditions (before anything moves)

1. [ ] Imaging/side-scan sonar confirmed disabled (SonarView extension not
       streaming; no traffic to/from its IP). It stays unused regardless.
2. [ ] Ping1D confirmed disabled unless explicitly requested for this session.
3. [ ] Manual override verified: pilot gamepad + Cockpit connected, DISARM
       reachable in one action, operator seated at controls.
4. [ ] Tether managed: free run length, no snag points, strain relief at both
       ends, tether does not bias left/right motion.
5. [ ] Pool clear: no people/animals/equipment in the water volume.
6. [ ] Control gains at conservative/low values; speed limits confirmed
       (≤0.3 m/s initial).
7. [ ] Depth check: pressure depth reads plausibly; V-bottom clearance along
       the planned line verified from the bottom survey.

## Manual phase

8. [ ] Manual low-speed motion in all axes (surge, sway, heave, yaw);
       verify response and neutral trim.
9. [ ] Coordinate-sign verification: for each axis, command a small positive
       input and record which way the vehicle ACTUALLY moves; reconcile with
       the software convention (x fwd, y left, z up) before any autonomy.

## Shadow phase (zero actuation)

10. [ ] Launch stack with `real_control_mode:=shadow` (defaults). Verify
        perception, tracker, planner run; `/real/shadow_cmd` shows sensible
        commands; interlock audit shows BLOCK entries only.
11. [ ] Operator comparison: pilot flies a slow approach toward the obstacle
        while the shadow planner runs; compare suggested vs expected commands.

## Live phase (requires explicit flags, only after 1–11 pass)

12. [ ] Enable autonomy at very low speed with
        `vehicle_in_water:=true allow_real_actuation:=true real_control_mode:=live`
        (transmit layer must have been implemented + reviewed by then).
13. [ ] Single straight-line transit WITHOUT obstacle; verify stop at end.
14. [ ] Emergency-stop test: operator disarm during an autonomous transit;
        measure stopping distance/behavior.
15. [ ] Only then: first obstacle trial (anchor, central approach, min speed).

Abort criteria at any step: unexpected motion direction, comms loss > 1 s,
depth excursion beyond bounds, tether snag, any leak warning → DISARM,
recover vehicle, log the event (failed runs are data, not embarrassments).
