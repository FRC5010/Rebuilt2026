# Hood / trench audit of autonomous routines

Check items off as they are fixed. Findings come from a static timeline analysis plus a headless
simulation of the real robot code.

## Terms and assumptions

- **Hood down** means hood angle <= 17.2 deg (low position 13.7 deg + the code's 3.5 deg tolerance).
  The real clearance angle under the trench beam is not known, so 20-24 deg is a flag, not proof of
  contact.
- **Trench stay** means the robot centre is within the structure's X extent (blue frame: x 4.02-5.22
  near, 11.31-12.51 far) and inside the corridor in Y (y < 1.28 bottom, y > 6.79 top).
- **Hood timing** comes from the simulation: a ~20 deg move takes ~0.65 s. It was not measured on
  hardware (no hood movement exists in the logs reviewed). Please measure it once on the robot.
- The hood lowering curve used (median, from ~35 deg): 0.3 s -> 25 deg, 0.5 s -> 19.5 deg,
  0.7 s -> 16 deg, 1.0 s -> 14 deg.
- Launcher starts the match in HAMMERTIME (hood down). `launcherPrep` raises it; `launcherLow` lowers it.

## Code-level issues (affect every auto)

- [x] No code protected the hood in auto. Fixed: `TrenchGuard` (velocity-aware, see `docs/AUTO_AUDIT.md`)
      drives the existing AUTO_HAMMERTIME state in teleop and auto. Sim result below. Still to do on the
      robot: measure the hood drop time (`TrenchGuard.HOOD_DROP_TIME_S`, 0.7 s from the sim) and check the
      beam clearance angle.
- [ ] Measure real hood travel time (PREP -> LOW and LOW -> PREP) and update the 0.7 s lead used here.

## Effect of the trench guard (simulation)

Worst hood angle under a beam, same sim, before -> after the guard (17.2 deg = down):

| Auto | Before | After |
| --- | --- | --- |
| PP `Orbit Right` | 25-31 | 14-19 |
| PP `Orbit Right 2 Swipe no HP` | 25-31 | 15-17 |
| PP `2X-TR-CTR-BUMP-DEP` | 25 | 13-14 |
| PP `TL-QTR-SS-QTRH-DEP-SS` (never lowers) | 28+ | 15-21 |
| PP `Follow Right Trench HP` | parked, 32 for 8 s | 18-19 |
| PP `Left 2056 double HP` | 32 | 14, 25 (two runs) |

Not yet cured: autos that start moving from rest within about 1 m of the beam (the `Follow ...`, `Quals 110`,
`Right 5010 Double*` family, 14 passes in the static guarded model). The guard cannot react before the robot
moves; lower the hood earlier in the auto. The checkboxes below are still the right list of autos to fix.

## PathPlanner autos (`src/main/deploy/pathplanner`)

Static filter: autos with a `launcherPrep` and no later `launcherLow` (do they ever lower?).
Only the starred one is a real hazard; the rest cross the trench only before prepping or never.

- [ ] `TL-QTR-SS-QTRH-DEP-SS`* (paths changed 4/26): prep @ 5.4 s, never lowers, passes 3 and 4 cross with
      hood 33-34 deg (sim: above 17.2 deg for 2.8 s and 6.9 s). Add `launcherLow` before passes 3 and 4.
- Not hazards: `A-TL-MS-DEP-MS`, `BR-HP-DEP-Wall`, `BRS-HP-DEP-TWR-HP`, `BRS-HP-DEP-TWR`,
  `BumpRight to wall`, `Delay Trench Neutral Bump HP`, `Follow Right Left Bumpt Depot`,
  `2X-TL-CTR-BUMP-DEP`, `A-BR-MS-HP-MS`, `A-BR-SS-HP-SS-Tested`, `A-HP-MS-DEP-MS`, `A-TR-MS-HP-MS`,
  `A-TR-SS-HP-SS`.

### Problem A: `launcherLow` immediately before a path that starts at the trench edge

Pattern: `launcherPrep`, `wait`, `launcherLow`, then a path starting at x ~ 3.5. The hood has 0.2-0.34 s
before the robot centre reaches the beam (static: 20-24 deg; sim: 21-23 deg). Fix: lower earlier (a path
marker, or `wait N-0.7`, `launcherLow`, `wait 0.7`).

- [ ] `Right 5010 Double`, `Right 5010 Double Old`, `Right 5010 Double Optimized`, `Right 5010 Double Short`
      (pass 3, 20 deg)
- [ ] `Left 2056 double HP` (pass 2, 24 deg; sim 32 deg for 0.4 s)
- [ ] `Follow Left Bump Depot middle end` (pass 1 21 deg, pass 2 24 deg)
- [ ] `Follow Left Bump Depot` (21 deg)
- [ ] `Follow Left Right Bump  HP` (21 deg)
- [ ] `Follow Left Trench ` (pass 1, 21 deg)
- [ ] `Follow Right Bump HP` (21 deg)
- [ ] `Quals 110` (pass 1, 21 deg)
- [ ] `Orbit Right` (pass 3, 20 deg)
- [ ] `Orbit Right 2 Swipe no HP` (pass 5, ~17 deg, borderline)
- [ ] `2X-TR-CTR-BUMP-DEP` (pass 2, 20 deg; sim 25 deg)
- [ ] `TR-CTR-SS-CTRH-MS-HP-SS` (pass 3, 20 deg)

### Problem B: `launcherPrep` marker fires at the trench exit edge (zero margin)

Hood passes 17.2 deg only 0.03-0.5 s after the robot centre leaves. Safe only with perfect tracking: PathPlanner
fires markers on trajectory time, so a lagging robot raises the hood while still in the trench (sim: 25-31 deg for
0.3-1.9 s on `Orbit Right` pass 4, `Orbit Right 2 Swipe no HP` pass 4, `Orbit Right 1Swipe`).
Fix: move the marker 1-2 m past the trench exit.

- [ ] `Orbit Right` (pass 2 0.50 s, pass 4 0.09 s)
- [ ] `Orbit Right 2 Swipe no HP` (pass 2 0.12 s, pass 4 0.03 s)
- [ ] `Orbit Right 1Swipe` (0.13 s)
- [ ] `Churn-Orbit Right 2 Swipe no HP` (0.12 s, 0.03 s)
- [ ] `Orbit Left` (0.18 s, 0.20 s)
- [ ] `Orbit Left 1Swipe` (0.17 s)
- [ ] `Left-3Shuttle-HPC` (0.50 s, 0.20 s)
- [ ] `Right-3Shuttle-HPC` (0.40 s, 0.28 s)
- [ ] `Follow Right Trench HP` (0.37 s; in sim the path ended with the robot parked in the trench, hood 32 deg
      for 8 s, because a path ends on its timer, not on arrival)
- [ ] `A-TR-CTRQ-HP` (0.50 s)
- [ ] `TL-DIS-NWALL-DEP` (0.17 s)
- [ ] `TL-QTR-SS-QTRH-DEP-SS` (0.18 s; also see the hazard above)
- [ ] `TR-CTR-SS-CTRH-MS-HP-SS` (0.50 s, 0.09 s)

### Clean (no hood problem found in static or sim)

`Left 5010 Double`, `Right 2056 double HP` (16 deg), `Quals 73` (15 deg static, 18 deg sim),
`Delay Trench Neutral Bump HP`, `Follow Right Left Bumpt Depot`, `A-TL-MS-DEP-MS`, and every auto that
never enters a trench. `Left 5010 Double` is a good pattern to copy: it puts `launcherLow` markers early
on the path rather than as a named command right before a path starts.

## How to re-check after a fix

Tests and tooling are now in the repo; see `docs/AUTO_AUDIT.md`.

- `./gradlew test --tests 'frc.robot.rebuilt.audit.*'` runs the static analysis. New hood-up trench passes
  fail the build; passes that no longer violate are printed as `FIXED since baseline`.
- `AUDIT_UPDATE_BASELINE=1 ./gradlew test --tests 'frc.robot.rebuilt.audit.AutoAuditBaselineTest'` rewrites
  `src/test/resources/auto-audit/baseline.json`; commit the smaller file and tick the item above.
  `build/auto-audit/report.md` is a generated checklist of every auto.
- `tools/auto-audit/run-sim-audit.sh <auto>` confirms in the headless simulation.
- The baseline also covers autos that were not part of the review above (older PathPlanner autos
  unchanged since 4/15); treat those as unreviewed findings.
- Pass criterion: hood <= 17.2 deg for the whole time the robot centre is inside a trench stay, with at
  least ~0.7 s between `launcherLow` and trench entry, and no `launcherPrep` until the robot has cleared
  the trench by about 0.5 s.

## Known limits of this audit

- The PathPlanner sim drivetrain lags its trajectory more than the real robot likely does, so sim numbers
  for PathPlanner autos are a stress test; the static numbers are the best case.
- Sim shot counts and final-approach timing are not reliable and were not evaluated.
