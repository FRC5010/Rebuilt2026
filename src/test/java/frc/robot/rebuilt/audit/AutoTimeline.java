package frc.robot.rebuilt.audit;

import java.util.List;

/**
 * An auto reduced to what the hood/trench analysis needs: when hood commands fire and where the
 * robot is over time (ideal path following).
 */
public final class AutoTimeline {
  /** robot position over time; returns {x, y} */
  public interface Position {
    double[] at(double t);
  }

  public final String name;
  public final double endTime;
  public final List<HoodCommand> hoodCommands;
  /** null if the auto never drives a path */
  public final Position position;

  public AutoTimeline(
      String name, double endTime, List<HoodCommand> hoodCommands, Position position) {
    this.name = name;
    this.endTime = endTime;
    this.hoodCommands =
        hoodCommands.stream().sorted((a, b) -> Double.compare(a.t(), b.t())).toList();
    this.position = position;
  }

  /** Static filter: is there a hood-lowering command after the first hood-raising command? */
  public LowerAfterRaise lowerAfterRaise() {
    int firstUp = -1;
    for (int i = 0; i < hoodCommands.size(); i++) {
      if (hoodCommands.get(i).up()) {
        firstUp = i;
        break;
      }
    }
    if (firstUp < 0) return LowerAfterRaise.NO_RAISE;
    for (int i = firstUp + 1; i < hoodCommands.size(); i++) {
      if (!hoodCommands.get(i).up()) return LowerAfterRaise.YES;
    }
    return LowerAfterRaise.NEVER_LOWERS;
  }

  public enum LowerAfterRaise {
    /** the auto never raises the hood */
    NO_RAISE,
    /** a launcherLow/launcherIdle follows the first launcherPrep */
    YES,
    /** the hood is raised and never lowered again */
    NEVER_LOWERS
  }
}
