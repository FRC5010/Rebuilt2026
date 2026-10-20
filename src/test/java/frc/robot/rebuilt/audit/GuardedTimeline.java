package frc.robot.rebuilt.audit;

import frc.robot.rebuilt.subsystems.Launcher.TrenchGuard;
import java.util.ArrayList;
import java.util.List;

/**
 * Rewrites an {@link AutoTimeline} the way the robot's trench guard would: the hood is held down
 * while {@link TrenchGuard} asks for it, and hood-raising commands that arrive meanwhile are
 * deferred until the guard has released (falling debounce included).
 *
 * <p>Velocity is the central difference of the ideal path position, so this is the best case for
 * the guard: real tracking lag shifts positions but not the lookahead logic. The unguarded timeline
 * says what the auto relies on the guard for.
 */
public final class GuardedTimeline {
  private GuardedTimeline() {}

  /** Must match Constants.Launcher.TRENCH_RELEASE_DEBOUNCE_S used by the real trigger. */
  static final double RELEASE_DEBOUNCE_S =
      frc.robot.rebuilt.Constants.Launcher.TRENCH_RELEASE_DEBOUNCE_S;

  public static AutoTimeline apply(AutoTimeline tl) {
    if (tl.position == null || tl.hoodCommands.stream().noneMatch(HoodCommand::up)) {
      return tl;
    }
    List<HoodCommand> out = new ArrayList<>();
    boolean effectiveUp = false;
    boolean desiredUp = false;
    int next = 0;
    double lastGuardTrue = Double.NEGATIVE_INFINITY;
    double dt = TrenchAudit.DT;
    for (double t = 0.0; t <= tl.endTime + 0.5; t += dt) {
      double commandT = Double.NaN;
      while (next < tl.hoodCommands.size() && tl.hoodCommands.get(next).t() <= t + 1e-9) {
        desiredUp = tl.hoodCommands.get(next).up();
        commandT = tl.hoodCommands.get(next).t();
        next++;
      }
      double[] p = tl.position.at(t);
      double[] a = tl.position.at(Math.max(0.0, t - dt));
      double[] b = tl.position.at(t + dt);
      double span = (t + dt) - Math.max(0.0, t - dt);
      double vx = (b[0] - a[0]) / span;
      double vy = (b[1] - a[1]) / span;
      // The real guard only evaluates while the hood is raised or already held; once the hold has
      // started it keeps evaluating.
      boolean holding = t - lastGuardTrue < RELEASE_DEBOUNCE_S;
      if ((desiredUp || holding) && TrenchGuard.shouldLowerHood(p[0], p[1], vx, vy)) {
        lastGuardTrue = t;
        holding = true;
      }
      boolean wantUp = desiredUp && !holding;
      if (wantUp != effectiveUp) {
        effectiveUp = wantUp;
        // a lowering caused by the auto's own command keeps that command's exact time
        double when = (!wantUp && !desiredUp && !Double.isNaN(commandT)) ? commandT : t;
        out.add(
            new HoodCommand(when, wantUp, wantUp ? "guardRelease" : "guardLower", "trench guard"));
      }
    }
    return new AutoTimeline(tl.name, tl.endTime, out, tl.position);
  }
}
