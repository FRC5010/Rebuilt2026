# Hood / trench audit of autonomous routines

Check items off as they are fixed. Findings come from a static timeline analysis plus a headless
simulation of the real robot code (October 2026, branch `b-line-implementation` at `a543865`).

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

- [ ] No code protects the hood in auto. `Launcher.setHoodAngle` documents a trench override but only
      forwards the request, and the auto-hammertime trigger in `LauncherCommands.configureButtonBindings`
      is commented out. Decide whether to restore a guard or rely on path design.
- [ ] `B_LineAutoCommands.java` registers `intakeRetracted` and `intakeRetracting` to
      `IntakeCommands.shouldIntaking()`. Unused by current paths, but any path using them would deploy
      the intake. Should be `shouldRetracted()` / `shouldRetracting()` (as in `NamedCommandsReg`).
- [x] Sim only: `AkitSwerveConfig` fed the real robot's kS (amps) into the sim as a friction voltage
      (4.15 V), so the sim drive undershot low speeds. Fixed: sim constants in `PhoenixUtil`
      (0.1 V drive / 0.05 V steer), guarded by `AkitSwerveSimFrictionTest`. Sim Square end miss
      0.218 m -> 0.002 m.
- [ ] Measure real hood travel time (PREP -> LOW and LOW -> PREP) and update the 0.7 s lead used here.

## BLine autos (`src/main/deploy/autos/paths`)

Hood-up trench crossings (confirmed in simulation, Blue and Red):

- [ ] `Right_5010_Double`: 3rd crossing (eastbound) with hood 33.5 deg in PREP. `launcherLow` (element 15)
      fires at about (7.4, 1.1), after the trench. Move it onto the segment leaving the shooting spot,
      `t_ratio` 0.0 (tested: hood down 1.2 s before the trench; at `t_ratio` 0.5 only 0.12 s).
- [ ] `left-5010-double`: 3rd crossing (top trench) with hood 33-34 deg. `launcherLow` (element 15) fires at
      about (8.0, 7.1), after the trench. Same fix (tested margin 0.86 s at `t_ratio` 0.0).
- [ ] `Right_2056_Double_HP`: 2nd crossing with hood 34 deg; the path has no `launcherLow` at all. Add one
      after waypoint (1.05, 1.40) at `t_ratio` 0.0 (tested margin 1.24 s).
- [ ] `Test: Intake + Shoot` (`Intake-Shoot`): ends inside the far trench with hood at 36 deg (test path).
- [ ] 5010 autos: first `launcherPrep` fires only ~0.07 m past the trench edge (x = 3.95). Works, but
      move it a little later on the segment for margin.
- [ ] Quals_73 raises the hood in the neutral zone and lowers it 1.1 s later (prep @ (8.0, 2.2), low @
      (6.7, 2.7)). Confirm this is intended.
- [ ] `Left_2056_Double_HP` has `launcherPrep` twice at the same spot. Remove the duplicate.

Clean for hood vs. trench: `Quals_73`, `Left_2056_Double_HP`.

Rotation targets (27 of 56 waypoint targets within 10 deg when passing, in sim):

- [ ] Rotation controller is P-only, kP = 1.0 (`GenericSwerveDrivetrain.setupBLine`), no feed-forward, so
      a heading ramp lags by about its rate in rad/s. Targets are met only at roughly <= 1 m/s.
      Consider a higher kP, rate feed-forward, or slower segments / earlier rotation elements.
      Worst: `Right_2056_Double_HP` 3/13, `Left_2056_Double_HP` 5/13, `left-5010-double` 7/12,
      `Right_5010_Double` 7/13, `Quals_73` 5/5.
- [ ] Follower finishes only when heading is within 2 deg; with kP = 1 the last degree creeps in (about
      5 s in one Red run). Consider a larger end rotation tolerance or minimum rotation speed.
- [ ] Intermediate waypoints use a 0.45 m handoff radius, so the robot turns about 0.4 m short of
      waypoints such as the shooting corner (1.16, 1.29). Use a smaller radius where position matters.
- [ ] Unexplained in sim: robot wedged ~10 s near (0.75, 0.8) on the final approach of
      `Right_5010_Double`. Cause not found; re-check on the real robot.
- [ ] Time budget: the repo documents AUTO as 20 s. Sim reached the final waypoint in about 5.5 s
      (Quals_73), 15.5 s (both 2056 autos), 17 s (Left 5010); Right 5010 was inconclusive. Verify on carpet.

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
- Red alliance was simulated for the five BLine competition autos only (same hood result as Blue).
- Sim shot counts and final-approach timing are not reliable and were not evaluated.
