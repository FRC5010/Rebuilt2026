package org.frc5010.common.sensors.camera;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Runs {@link QuestNavInterface} against {@link QuestNavSim} through the real QuestNav library, the
 * same path the robot uses in simulation.
 */
class QuestNavInterfaceTest {
  /** Same mount as deploy/rebuilt_robot/cameras/quest.json. */
  private static final Transform3d MOUNT =
      new Transform3d(-0.171968, -0.252501, 0.225336, new Rotation3d(0, 0, Math.PI));

  private static final double EPS = 1e-3;

  private Pose2d truePose = new Pose2d(2, 3, Rotation2d.fromDegrees(30));
  private QuestNavInterface quest;

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0));
    // Headset world origin rotated and offset from the field, as after booting somewhere random.
    SmartDashboard.putNumber("QuestNavSim/OriginX", 1.5);
    SmartDashboard.putNumber("QuestNavSim/OriginY", -0.7);
    SmartDashboard.putNumber("QuestNavSim/OriginDeg", 73);
    quest = new QuestNavInterface(MOUNT).withSimulation(() -> truePose);
    awaitTrue(() -> quest.getSim().orElseThrow().isHeadsetConnected(), "headset should connect");
    awaitTrue(
        () -> {
          quest.update();
          return quest.isConnected();
        },
        "library should see the headset as connected");
  }

  @AfterEach
  void tearDown() {
    quest.getSim().ifPresent(QuestNavSim::close);
  }

  private static void awaitTrue(BooleanSupplier condition, String message) {
    long deadline = System.nanoTime() + 5_000_000_000L;
    boolean ok = condition.getAsBoolean();
    while (!ok && System.nanoTime() < deadline) {
      try {
        Thread.sleep(5);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        break;
      }
      ok = condition.getAsBoolean();
    }
    assertTrue(ok, message);
  }

  /** Keeps updating until the reported pose matches the true pose. */
  private void awaitPoseMatchesTruth() {
    awaitTrue(
        () -> {
          quest.update();
          return quest
              .getRobotPose()
              .map(Pose3d::toPose2d)
              .map(
                  p ->
                      p.getTranslation().getDistance(truePose.getTranslation()) < EPS
                          && Math.abs(p.getRotation().minus(truePose.getRotation()).getRadians())
                              < EPS)
              .orElse(false);
        },
        "reported pose should track the true pose");
  }

  @Test
  void softResetAlignsHeadsetFrameAndTracksMotion() {
    // Align to the starting pose as the estimator does while disabled. resetPose() is a no-op
    // while the headset looks disconnected, so retry it like the estimator's per-loop reset.
    awaitTrue(
        () -> {
          quest.update();
          quest.resetPose(new Pose3d(truePose));
          return quest.isActive();
        },
        "headset should be active once connected, tracking, and reset");
    awaitPoseMatchesTruth();

    // Drive somewhere else, turning; the soft reset must hold in the rotated headset frame.
    truePose = new Pose2d(5.2, 1.1, Rotation2d.fromDegrees(-120));
    awaitPoseMatchesTruth();

    Pose2d reported = quest.getRobotPose().orElseThrow().toPose2d();
    assertEquals(5.2, reported.getX(), EPS);
    assertEquals(1.1, reported.getY(), EPS);
    assertEquals(-120, reported.getRotation().getDegrees(), 0.1);
  }
}
