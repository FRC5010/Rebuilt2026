package frc.robot.rebuilt.audit;

import java.util.List;

/** One continuous period with the robot under a trench structure, and the hood during it. */
public final class Stay {
  public final String zone;
  public final double t0;
  public double t1;
  public final double x0;
  public final double y0;
  public final double angleAtEntry;
  public double maxAngle;
  /**
   * If the hood is raised near the end of the stay: seconds after leaving until the hood passes the
   * down threshold. NaN if not applicable. Small values mean a PREP that fires right at the trench
   * exit (safe only with perfect tracking).
   */
  public double exitMarginSeconds = Double.NaN;

  public HoodCommand lastCommandBeforeEntry;
  public List<HoodCommand> commandsInside = List.of();

  Stay(String zone, double t0, double x0, double y0, double angle) {
    this.zone = zone;
    this.t0 = t0;
    this.x0 = x0;
    this.y0 = y0;
    this.angleAtEntry = angle;
    this.maxAngle = angle;
  }

  public Severity severity() {
    return Severity.of(maxAngle);
  }

  public boolean violation() {
    return severity() != Severity.OK;
  }

  @Override
  public String toString() {
    return String.format(
        "%s t=%.1f-%.1fs at (%.1f,%.1f) hood %.0f deg at entry, max %.0f deg -> %s",
        zone, t0, t1, x0, y0, angleAtEntry, maxAngle, severity());
  }
}
