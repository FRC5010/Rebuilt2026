package frc.robot.rebuilt.audit;

import java.util.ArrayList;
import java.util.List;

/** Finds every trench stay in an {@link AutoTimeline} and rates the hood during it. */
public final class TrenchAudit {
  private TrenchAudit() {}

  public static final double DT = 0.02;

  /**
   * @param margin {@link TrenchGeometry#CENTER} or {@link TrenchGeometry#BODY}
   * @return stays in time order (empty if the auto has no path)
   */
  public static List<Stay> stays(AutoTimeline tl, double margin) {
    List<Stay> out = new ArrayList<>();
    if (tl.position == null) return out;
    HoodModel.Trace hood = HoodModel.trace(tl.hoodCommands);
    Stay cur = null;
    for (double t = 0.0; t <= tl.endTime + 0.5; t += DT) {
      double[] p = tl.position.at(t);
      String z = TrenchGeometry.zone(p[0], p[1], margin);
      double a = hood.angleAt(t);
      if (z != null && (cur == null || !cur.zone.equals(z))) {
        if (cur != null) {
          cur.t1 = t;
          out.add(cur);
        }
        cur = new Stay(z, t, p[0], p[1], a);
      } else if (z == null && cur != null) {
        cur.t1 = t;
        out.add(cur);
        cur = null;
      }
      if (cur != null && a > cur.maxAngle) cur.maxAngle = a;
    }
    if (cur != null) {
      cur.t1 = tl.endTime + 0.5;
      out.add(cur);
    }
    for (Stay s : out) annotate(tl, hood, s);
    return out;
  }

  private static void annotate(AutoTimeline tl, HoodModel.Trace hood, Stay s) {
    HoodCommand last = null;
    List<HoodCommand> inside = new ArrayList<>();
    for (HoodCommand c : tl.hoodCommands) {
      if (c.t() <= s.t0 + 1e-6) last = c;
      else if (c.t() <= s.t1) inside.add(c);
    }
    s.lastCommandBeforeEntry = last;
    s.commandsInside = inside;
    boolean raisedNearby =
        tl.hoodCommands.stream()
            .anyMatch(c -> c.up() && c.t() >= s.t0 - 1.0 && c.t() <= s.t1 + 1.0);
    if (raisedNearby && s.maxAngle <= HoodModel.DOWN_THRESHOLD_DEG) {
      double t = s.t0;
      double end = s.t1 + 2.0;
      while (t < end && hood.angleAt(t) <= HoodModel.DOWN_THRESHOLD_DEG) t += DT;
      if (t < end) s.exitMarginSeconds = t - s.t1;
    }
  }

  /** Number of the stay (1-based) in the given list, for reporting "pass N". */
  public static List<Integer> violationPasses(List<Stay> stays) {
    List<Integer> out = new ArrayList<>();
    for (int i = 0; i < stays.size(); i++) if (stays.get(i).violation()) out.add(i + 1);
    return out;
  }
}
