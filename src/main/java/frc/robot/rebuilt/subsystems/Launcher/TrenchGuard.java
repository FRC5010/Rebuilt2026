package frc.robot.rebuilt.subsystems.Launcher;

import frc.robot.rebuilt.FieldConstants;

/**
 * Decides when the launcher hood has to come down for a trench, using the robot's position and its
 * field velocity.
 *
 * <p>The hood must be down while the robot is under a trench. A raised hood needs about {@link
 * #HOOD_DROP_TIME_S} to come down, so the guard looks ahead: it extends a segment from the robot
 * along its velocity by {@link #triggerDistance(double)} and asks for the hood to lower if that
 * segment touches a trench. A stationary robot outside a trench is never flagged, which lets the
 * driver stop close to the structure and shoot.
 *
 * <p>All coordinates are the blue-origin field frame used by the pose estimator. The four trenches
 * are physical field features, so nothing here depends on the alliance.
 */
public final class TrenchGuard {
  private TrenchGuard() {}

  /**
   * Conservative time for the hood to go from fully raised to the "down" threshold. Measured in the
   * headless simulation (about 0.6 s to 17.2 degrees from 35 degrees, 0.7 s used). Not yet measured
   * on the robot.
   */
  public static final double HOOD_DROP_TIME_S = 0.70;

  /** State machine transition plus hood command latency, in addition to the drop time. */
  public static final double REACTION_LATENCY_S = 0.10;

  /**
   * Extra distance allowance for a robot that speeds up toward the trench after the hood has been
   * commanded down, as an acceleration in m/s^2. A robot that starts from rest next to the beam and
   * accelerates harder than this can still enter before the hood is down.
   */
  public static final double ACCEL_ALLOWANCE_MPS2 = 2.0;

  /**
   * The acceleration allowance fades in linearly up to this speed, so that a robot creeping or
   * jittering at a few cm/s next to a trench does not trip the guard.
   */
  public static final double ACCEL_ALLOWANCE_FULL_SPEED_MPS = 1.0;

  /** Below this speed the robot is treated as stationary. */
  public static final double MIN_SPEED_MPS = 0.1;

  /** Extra clearance around the structure's footprint in X, for hood position and pose error. */
  public static final double STRUCTURE_MARGIN_M = 0.10;

  private static final double HALF_DEPTH = FieldConstants.LeftTrench.depth / 2.0;
  private static final double OPENING = FieldConstants.LeftTrench.openingWidth;
  private static final double FIELD_WIDTH = FieldConstants.fieldWidth;

  private static final double[] TRENCH_CENTER_X = {
    FieldConstants.LinesVertical.hubCenter, FieldConstants.LinesVertical.oppHubCenter
  };

  /**
   * Distance ahead of the robot that must be clear of trench for the hood to stay up.
   *
   * @param speedMps magnitude of the field velocity in m/s
   * @return the lookahead in metres, 0 when the robot is (nearly) stationary
   */
  public static double triggerDistance(double speedMps) {
    if (speedMps < MIN_SPEED_MPS) {
      return 0.0;
    }
    double t = HOOD_DROP_TIME_S + REACTION_LATENCY_S;
    double fade = Math.min(1.0, speedMps / ACCEL_ALLOWANCE_FULL_SPEED_MPS);
    return speedMps * t + fade * 0.5 * ACCEL_ALLOWANCE_MPS2 * t * t;
  }

  /**
   * @param x robot x in the blue-origin field frame, metres
   * @param y robot y in the blue-origin field frame, metres
   * @param vx field-relative x velocity, m/s
   * @param vy field-relative y velocity, m/s
   * @return true if the robot is under a trench or will be before the hood can come down
   */
  public static boolean shouldLowerHood(double x, double y, double vx, double vy) {
    double speed = Math.hypot(vx, vy);
    double reach = triggerDistance(speed);
    double dx = speed > 0 ? vx / speed * reach : 0.0;
    double dy = speed > 0 ? vy / speed * reach : 0.0;
    for (double cx : TRENCH_CENTER_X) {
      double minX = cx - HALF_DEPTH - STRUCTURE_MARGIN_M;
      double maxX = cx + HALF_DEPTH + STRUCTURE_MARGIN_M;
      if (segmentHits(x, y, dx, dy, minX, maxX, 0.0, OPENING)
          || segmentHits(x, y, dx, dy, minX, maxX, FIELD_WIDTH - OPENING, FIELD_WIDTH)) {
        return true;
      }
    }
    return false;
  }

  /** Segment (x, y) to (x + dx, y + dy) against an axis-aligned rectangle (slab method). */
  private static boolean segmentHits(
      double x,
      double y,
      double dx,
      double dy,
      double minX,
      double maxX,
      double minY,
      double maxY) {
    double t0 = 0.0;
    double t1 = 1.0;
    double[] origin = {x, y};
    double[] delta = {dx, dy};
    double[] lo = {minX, minY};
    double[] hi = {maxX, maxY};
    for (int i = 0; i < 2; i++) {
      if (Math.abs(delta[i]) < 1e-12) {
        if (origin[i] < lo[i] || origin[i] > hi[i]) {
          return false;
        }
      } else {
        double a = (lo[i] - origin[i]) / delta[i];
        double b = (hi[i] - origin[i]) / delta[i];
        t0 = Math.max(t0, Math.min(a, b));
        t1 = Math.min(t1, Math.max(a, b));
        if (t0 > t1) {
          return false;
        }
      }
    }
    return true;
  }
}
