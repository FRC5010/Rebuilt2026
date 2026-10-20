package frc.robot.rebuilt.audit;

import java.util.ArrayList;
import java.util.List;

/**
 * Hood angle versus time, built from the headless simulation (median curves).
 *
 * <p>DOWN: seconds after {@code launcherLow} starting from ~35 deg. UP: seconds after {@code
 * launcherPrep} starting from the low position. A command given while the hood is mid-travel
 * continues from the current angle. The real hood speed has not been measured on hardware; update
 * the curves when it has.
 */
public final class HoodModel {
  public static final double LOW_DEG = 12.8;
  /** Hood counts as "down" at or below this (low position + the code's 3.5 deg tolerance). */
  public static final double DOWN_THRESHOLD_DEG = 17.2;

  private static final double[][] DOWN = {
    {0.0, 35.1}, {0.1, 35.2}, {0.2, 30.7}, {0.3, 25.5}, {0.4, 21.9}, {0.5, 19.5},
    {0.6, 17.5}, {0.7, 16.2}, {0.8, 15.2}, {1.0, 13.8}, {1.2, 13.1}, {1.5, 12.8}
  };
  private static final double[][] UP = {
    {0.0, 12.8},
    {0.1, 12.9},
    {0.2, 18.8},
    {0.3, 23.3},
    {0.4, 25.9},
    {0.5, 28.1},
    {0.6, 30.0},
    {0.7, 31.0},
    {0.8, 31.6},
    {1.0, 33.0},
    {1.3, 33.5}
  };

  private HoodModel() {}

  static double interp(double[][] c, double x) {
    if (x <= c[0][0]) return c[0][1];
    for (int i = 0; i + 1 < c.length; i++) {
      if (x <= c[i + 1][0]) {
        double f = (x - c[i][0]) / (c[i + 1][0] - c[i][0]);
        return c[i][1] + f * (c[i + 1][1] - c[i][1]);
      }
    }
    return c[c.length - 1][1];
  }

  /** time on a monotone curve at which it reaches value y */
  static double inverse(double[][] c, double y) {
    boolean inc = c[c.length - 1][1] > c[0][1];
    if ((inc && y <= c[0][1]) || (!inc && y >= c[0][1])) return 0.0;
    for (int i = 0; i + 1 < c.length; i++) {
      double ya = c[i][1], yb = c[i + 1][1];
      double lo = inc ? ya : yb, hi = inc ? yb : ya;
      if (y >= lo && y <= hi && ya != yb) {
        return c[i][0] + (c[i + 1][0] - c[i][0]) * (y - ya) / (yb - ya);
      }
    }
    return c[c.length - 1][0];
  }

  /** Hood angle as a function of time for a time-ordered list of hood commands. */
  public static final class Trace {
    private record Seg(double tStart, boolean up, double tau0, double tEnd, boolean initial) {}

    private final List<Seg> segs = new ArrayList<>();

    Trace(List<HoodCommand> commands) {
      List<HoodCommand> sorted = new ArrayList<>(commands);
      sorted.sort((a, b) -> Double.compare(a.t(), b.t()));
      boolean curUp = false;
      double curT = Double.NEGATIVE_INFINITY, curTau = 99.0;
      boolean initial = true;
      for (HoodCommand c : sorted) {
        double h = initial ? LOW_DEG : angle(curUp, curT, curTau, c.t());
        segs.add(new Seg(curT, curUp, curTau, c.t(), initial));
        curUp = c.up();
        curT = c.t();
        curTau = inverse(curUp ? UP : DOWN, h);
        initial = false;
      }
      segs.add(new Seg(curT, curUp, curTau, Double.POSITIVE_INFINITY, initial));
    }

    private static double angle(boolean up, double tStart, double tau0, double t) {
      return interp(up ? UP : DOWN, tau0 + (t - tStart));
    }

    public double angleAt(double t) {
      for (Seg s : segs) {
        if (t >= s.tStart() && t < s.tEnd()) {
          return s.initial() ? LOW_DEG : angle(s.up(), s.tStart(), s.tau0(), t);
        }
      }
      return LOW_DEG;
    }
  }

  public static Trace trace(List<HoodCommand> commands) {
    return new Trace(commands);
  }
}
