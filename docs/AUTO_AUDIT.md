# Auto audit: hood vs. trench

Repeatable checks that the launcher hood is down whenever the robot is under a trench beam during an
autonomous routine, for both B-Line paths (`src/main/deploy/autos/paths`) and PathPlanner autos
(`src/main/deploy/pathplanner`). The findings that motivated this are in `HOOD_TRENCH_AUDIT.md`.

## What runs

| Check | Command | Runs in `./gradlew test`? |
| --- | --- | --- |
| Unit tests (hood model, geometry, both analyzers on synthetic fixtures) | `./gradlew test --tests 'frc.robot.rebuilt.audit.*'` | yes |
| Static ratchet over every real auto | `AutoAuditBaselineTest` | yes |
| Sim friction regression | `AkitSwerveSimFrictionTest` | yes |
| Headless full-robot simulation of one auto | `tools/auto-audit/run-sim-audit.sh` | no (opt-in, about 25 s per auto) |

The static ratchet writes `build/auto-audit/report.md`: a checklist of every auto with its trench passes,
the hood angle at each, and whether it lowers the hood after raising it.

## How the static analysis works

For each auto it builds a timeline of hood commands (`launcherPrep`/`Preset` raise, `launcherLow`/`Idle`
lower) and the robot's ideal position over time, then walks the timeline in 20 ms steps, tracking the hood
angle with `HoodModel` and noting every "stay" with the robot centre under a trench structure.

* Timing is the fastest plausible case (so lead times are conservative): PathPlanner paths use a
  rest-to-rest trapezoid from the path's max velocity/acceleration; B-Line segments use the larger of the two
  adjacent speed limits. Tracking lag, constraint zones and the B-Line handoff radius are ignored.
* PathPlanner fires event markers and ends paths on trajectory time, so a robot that lags its trajectory
  is later at each marker's location than the static model assumes. The simulation shows this.
* Trench geometry comes from `FieldConstants`; the bumper size from `akit_swerve_drivetrain.json`.
* The hood must be at or below 17.2 deg (low position + the 3.5 deg tolerance used by the launcher).
  Severity: OK, MARGINAL (to 22 deg), POOR (to 28 deg), FAIL.
* The hood curves in `HoodModel` come from the simulation, not hardware. Measure PREP-to-LOW and
  LOW-to-PREP on the robot and update them; the whole audit then re-calibrates.

## Workflow for fixing an issue

1. Edit the auto (for example, move `launcherLow` earlier).
2. `./gradlew test --tests 'frc.robot.rebuilt.audit.*'`. The output lists `FIXED since baseline`
   entries for passes that no longer violate. A pass that newly violates fails the build.
3. `AUDIT_UPDATE_BASELINE=1 ./gradlew test --tests 'frc.robot.rebuilt.audit.AutoAuditBaselineTest'`
   rewrites the static section of `src/test/resources/auto-audit/baseline.json`. Commit the smaller file;
   the diff is the list of fixes. Tick the item in `HOOD_TRENCH_AUDIT.md`.
4. Confirm in simulation: `tools/auto-audit/run-sim-audit.sh --update-baseline "Right 5010 Double"`
   (PathPlanner auto name or B-Line path name, such as `Right_5010_Double`).

`AUDIT_STRICT_BASELINE=1` makes both the static and sim tests fail when a baseline entry no longer
reproduces, which is useful in CI to force the baseline to shrink.

## Simulation runner

```
tools/auto-audit/run-sim-audit.sh [--kind bline|pathplanner|all] [--alliance Blue|Red]
                                  [--seconds N] [--repeat N] [--update-baseline] [--strict] [name ...]
```

Each auto runs in a fresh JVM (the robot is a singleton). The test boots the real `Robot` under the HAL
simulator, selects the auto on the dashboard chooser, places the robot at the start pose and steps
simulated time. Set `AUDIT_OUT=/path/file.csv` for a per-tick trace (pose, hood angle, launcher state).

**What is enforced.** B-Line sim results are repeatable (the worst hood angle under a beam differed by at
most 0.4 deg between identical runs), so they are checked against the `sim` section of the baseline
(violation = hood above 22 deg under a beam). PathPlanner sim results are *not* repeatable (39 of 60 autos
reproduced within 0.5 deg, but others differed by 5-17 deg between identical runs, because the sim robot
lags its trajectory and PathPlanner ends paths and fires markers on trajectory time). They are printed as a
stress test and not enforced; the deterministic static analysis is the PathPlanner gate. Use `--repeat N`
to see the spread, or set `AUDIT_SIM_ENFORCE_PATHPLANNER=1` to enforce anyway. The cause of the
run-to-run variation was not isolated.

## Files

* `src/test/java/frc/robot/rebuilt/audit/` : `HoodModel`, `TrenchGeometry`, `TrenchAudit`, `PathPlannerAudit`,
  `BLineAudit`, `AuditBaseline`, the tests, and `AutoAuditSimTest`
* `src/test/resources/auto-audit/fixtures/` : synthetic autos with known answers
* `src/test/resources/auto-audit/baseline.json` : known violations (the ratchet)
* `tools/auto-audit/run-sim-audit.sh`
