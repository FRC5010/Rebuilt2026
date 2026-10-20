package frc.robot.rebuilt.audit;

/** How bad the hood angle is while the robot is under a trench beam. */
public enum Severity {
  OK,
  MARGINAL,
  POOR,
  FAIL;

  public static Severity of(double maxHoodDeg) {
    if (maxHoodDeg <= HoodModel.DOWN_THRESHOLD_DEG) return OK;
    if (maxHoodDeg <= 22) return MARGINAL;
    if (maxHoodDeg <= 28) return POOR;
    return FAIL;
  }
}
