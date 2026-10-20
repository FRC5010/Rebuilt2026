package frc.robot.rebuilt.subsystems.Launcher;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.rebuilt.FieldConstants;
import org.junit.jupiter.api.Test;

/** Behaviour of the velocity-aware trench guard (blue-origin field coordinates throughout). */
class TrenchGuardTest {
  private static final double HUB_X = FieldConstants.LinesVertical.hubCenter;
  private static final double OPP_HUB_X = FieldConstants.LinesVertical.oppHubCenter;
  private static final double HALF_DEPTH = FieldConstants.LeftTrench.depth / 2.0;
  private static final double OPENING = FieldConstants.LeftTrench.openingWidth;
  private static final double WIDTH = FieldConstants.fieldWidth;

  /** x of the near edge of the near-trench structure, as seen from the alliance wall */
  private static final double EDGE_X = HUB_X - HALF_DEPTH - TrenchGuard.STRUCTURE_MARGIN_M;

  @Test
  void stationaryOutsideTheTrenchNeverTriggers() {
    // the driver may sit right next to the beam and shoot
    assertFalse(TrenchGuard.shouldLowerHood(EDGE_X - 0.01, 0.6, 0, 0));
    assertFalse(TrenchGuard.shouldLowerHood(EDGE_X - 0.01, WIDTH - 0.6, 0, 0));
  }

  @Test
  void stationaryInsideTheTrenchAlwaysTriggers() {
    assertTrue(TrenchGuard.shouldLowerHood(HUB_X, 0.6, 0, 0));
    assertTrue(TrenchGuard.shouldLowerHood(OPP_HUB_X, WIDTH - 0.6, 0, 0));
  }

  @Test
  void drivingTowardTheTrenchTriggersAtTheLookaheadDistance() {
    double speed = 3.0;
    double lookahead = TrenchGuard.triggerDistance(speed);
    assertTrue(lookahead > speed * TrenchGuard.HOOD_DROP_TIME_S, "must exceed the hood drop time");
    assertTrue(TrenchGuard.shouldLowerHood(EDGE_X - lookahead + 0.05, 0.6, speed, 0));
    assertFalse(TrenchGuard.shouldLowerHood(EDGE_X - lookahead - 0.05, 0.6, speed, 0));
  }

  @Test
  void lookaheadGrowsWithSpeedAndIsZeroWhenStopped() {
    assertEquals(0.0, TrenchGuard.triggerDistance(0.0), 1e-12);
    assertEquals(0.0, TrenchGuard.triggerDistance(TrenchGuard.MIN_SPEED_MPS * 0.5), 1e-12);
    assertTrue(TrenchGuard.triggerDistance(1.0) < TrenchGuard.triggerDistance(2.0));
  }

  @Test
  void creepingNextToTheTrenchDoesNotTrigger() {
    // a robot waiting at a shooting spot 0.3 m from the beam, jittering at a few cm/s
    assertFalse(TrenchGuard.shouldLowerHood(EDGE_X - 0.3, 0.6, 0.15, 0));
    assertTrue(TrenchGuard.triggerDistance(0.15) < 0.3);
  }

  @Test
  void lookaheadIsContinuousAndMonotonic() {
    double prev = 0.0;
    for (double v = TrenchGuard.MIN_SPEED_MPS; v <= 5.0; v += 0.05) {
      double d = TrenchGuard.triggerDistance(v);
      assertTrue(d >= prev, "speed " + v);
      prev = d;
    }
  }

  @Test
  void movingAwayOrAlongTheTrenchDoesNotTrigger() {
    double x = EDGE_X - 0.3;
    assertFalse(TrenchGuard.shouldLowerHood(x, 0.6, -3.0, 0), "driving away");
    assertFalse(
        TrenchGuard.shouldLowerHood(x, 3.0, 0, 3.0), "driving along the field in the middle");
    // y component alone moving toward the corridor but x is outside the structure
    assertFalse(TrenchGuard.shouldLowerHood(x, 2.5, 0, -3.0));
  }

  @Test
  void approachingTheCorridorFromTheFieldMiddleTriggers() {
    // directly above the near trench, heading to the wall at 3 m/s
    double d = TrenchGuard.triggerDistance(3.0);
    assertTrue(TrenchGuard.shouldLowerHood(HUB_X, OPENING + d - 0.05, 0, -3.0));
    assertFalse(TrenchGuard.shouldLowerHood(HUB_X, OPENING + d + 0.05, 0, -3.0));
    assertTrue(TrenchGuard.shouldLowerHood(HUB_X, WIDTH - OPENING - d + 0.05, 0, 3.0));
  }

  @Test
  void allFourTrenchesAreGuardedAndAreAllianceIndependent() {
    double d = TrenchGuard.triggerDistance(3.0);
    double[] xs = {HUB_X, OPP_HUB_X};
    double[] ys = {0.6, WIDTH - 0.6};
    for (double cx : xs) {
      for (double y : ys) {
        assertTrue(TrenchGuard.shouldLowerHood(cx - HALF_DEPTH - 0.5, y, 3.0, 0), "from the left");
        assertTrue(
            TrenchGuard.shouldLowerHood(cx + HALF_DEPTH + 0.5, y, -3.0, 0), "from the right");
        assertFalse(TrenchGuard.shouldLowerHood(cx - HALF_DEPTH - d - 0.5, y, 3.0, 0));
      }
    }
  }

  @Test
  void diagonalApproachUsesTheVelocityDirection() {
    double speed = 3.0;
    double d = TrenchGuard.triggerDistance(speed);
    double c = Math.sqrt(0.5);
    // 45 degrees toward the bottom near trench; start 0.5*d away along each axis
    double x = EDGE_X - 0.5 * d * c;
    double y = OPENING + 0.5 * d * c;
    assertTrue(TrenchGuard.shouldLowerHood(x, y, speed * c, -speed * c));
    // same position but heading the other way
    assertFalse(TrenchGuard.shouldLowerHood(x, y, -speed * c, speed * c));
  }

  @Test
  void guardIsConservativeForTheHoodDropTime() {
    // At trigger the robot still needs lookahead metres: at constant speed that takes at least the
    // hood drop time plus the reaction latency.
    for (double v = 0.5; v <= 5.0; v += 0.5) {
      double seconds = TrenchGuard.triggerDistance(v) / v;
      assertTrue(
          seconds >= TrenchGuard.HOOD_DROP_TIME_S + TrenchGuard.REACTION_LATENCY_S - 1e-9,
          "speed " + v);
    }
  }
}
