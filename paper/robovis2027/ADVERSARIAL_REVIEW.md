# Final adversarial review

This internal review is not part of the submission PDF.

## Reviewer 1 — underwater robotics

**Plausible rejection case.** The physical campaign has eight admissible runs
in one pool, on one day, around one static anchor; K1/K1M have only two runs
each. There is no Doppler velocity log, acoustic positioning, or overhead
tracking during the avoidance runs, and only the committed planner flew.
Consequently, the paper cannot establish physical clearance, trajectory
accuracy, collision-free geometry, or a real-world planner comparison.

**Disposition.** These are irreducible limitations, not prose defects. The
abstract, physical-results section, and limitations now restrict the physical
claim to recorded command-domain behaviour and say that no collision was
recorded rather than that avoidance was spatially validated.

## Reviewer 2 — sim-to-real methodology

**Plausible rejection case.** Deployment-time parameter changes weaken a
nominally frozen study, while the 80-run ablation was designed after observing
the pool outcome. A7 could be mistaken for the deployed stack even though it
contains the unflown A5 FOV correction and A6 geometry rescaling. A6 could be
over-read as showing that every basin-compatible manoeuvre is unsafe.

**Disposition.** The manuscript uses explicit PRE→REAL→POST chronology. A0 is
the same-session diagnostic baseline; A1--A6 are single-factor changes; A7 is
labelled *combined diagnostic configuration* and *not flown*. A6 is scoped to
one tested rescaling. No post-deployment result is called a prediction.

## Reviewer 3 — ROBOVIS generalist

**Plausible rejection case.** A dense provenance narrative may obscure the
central contribution, and inconsistent labels or an oversized table could
make the experiment difficult to reconstruct. Double-blind leakage or an
unexplained AI acknowledgement could also make the file non-compliant.

**Disposition.** The abstract and introduction now state one central result,
the contribution list is shorter, terminology is locked, the A-table is
generated from committed artifacts, and oversized tables are constrained to
the text width. The review build has anonymous metadata and no credits;
required AI disclosure is held separately for the camera-ready stage.

## Recommendation after corrections

**Borderline accept on transparency and methodological value, not on
algorithmic novelty.** The remaining weaknesses are empirical scope and
missing physical spatial ground truth. They cannot be repaired without new
experiments and are therefore stated prominently rather than disguised.
