# Terminology and style lock

The manuscript uses British English, matching *manoeuvre*, *behaviour*, and
*randomisation*. Code keys and artifact filenames retain their original
spelling when quoted literally.

| Preferred term | Meaning and use |
|---|---|
| deployment gap | The documented differences between the frozen simulated stack and the stack that flew. |
| frozen campaign | The 80 simulations executed, hashed, and committed before physical deployment. |
| pre-deployment (`PRE`) | Evidence fixed before observing the physical campaign. |
| physical (`REAL`) | Evidence acquired in the pool; never treated as spatial ground truth. |
| post-deployment diagnostic (`POST`) | Simulations or audits performed after physical outcomes were known. |
| committed planner | The finite-state controller flown in the admissible physical runs. |
| dynamic-window baseline | The holonomic DWA evaluated only in simulation; not a physically tested planner. |
| calibration level | One of S0--S3 in the frozen campaign. |
| admissible run | A physical run passing the pre-specified command-source contamination criterion. |
| excluded run | A run failing that criterion; avoid the ambiguous phrase *technically invalid*. |
| engagement | Entry into the planner's avoidance behaviour at the configured range. |
| strafe | The first lateral phase of the committed planner. |
| go-around | The second manoeuvre phase, identified from its command setpoint. |
| command domain | The shared planner-output boundary used for the defensible sim-to-real comparison. |
| spatial ground truth | Externally referenced position or trajectory measurements; unavailable during the physical avoidance runs. |

Use *tested pool-feasible rescaling* for A6: it is one tested parameterisation,
not an exhaustive basin feasibility result. Use *combined diagnostic
configuration* for A7: it combines A1--A6, including the unflown A5/A6
changes, and must never be labelled *deployed* or *as flown*.
