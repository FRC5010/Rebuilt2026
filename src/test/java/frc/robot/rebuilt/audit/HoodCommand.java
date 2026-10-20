package frc.robot.rebuilt.audit;

import java.util.Set;

/** A command that raises or lowers the launcher hood, at an (estimated) time in an auto. */
public record HoodCommand(double t, boolean up, String name, String source) {
  /** Auto command names that leave the hood raised (PREP / PRESET states). */
  public static final Set<String> UP_NAMES =
      Set.of("launcherPrep", "launcherPreset", "hubPreset", "towerPreset", "towerForwardPreset");

  /** Auto command names that lower the hood (LOW_SPEED / IDLE states). */
  public static final Set<String> DOWN_NAMES = Set.of("launcherLow", "launcherIdle");

  /**
   * @return a HoodCommand if the name affects the hood, otherwise null
   */
  public static HoodCommand ofName(double t, String name, String source) {
    if (UP_NAMES.contains(name)) return new HoodCommand(t, true, name, source);
    if (DOWN_NAMES.contains(name)) return new HoodCommand(t, false, name, source);
    return null;
  }

  @Override
  public String toString() {
    return String.format("%s@%.2fs", name.replace("launcher", ""), t);
  }
}
