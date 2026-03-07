// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.pose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import java.util.Arrays;
import java.util.List;
import org.frc5010.common.vision.VisionConstants;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface PoseProvider {

  public VisionIOInputsAutoLogged input = new VisionIOInputsAutoLogged();
  public Alert disconnectedAlert = new Alert("PoseProvider", AlertType.kWarning);
  public int cameraIndex = 0;

  public enum ProviderType {
    ALL,
    NONE,
    FIELD_BASED,
    ENVIRONMENT_BASED,
    RELATIVE
  }

  public enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION,
    ENVIRONMENT_BASED,
  }

  /**
   * The algorithm used to compute a pose estimate from one or more fiducial targets.
   *
   * <p>PhotonVision-specific values ({@link #MULTI_TAG_PNP}, {@link #SINGLE_TAG_PNP}, {@link
   * #TRIG_PNP}) are set directly from the pipeline result. {@link #UNKNOWN} is used for providers
   * (Limelight, QuestNav) that do not expose this information.
   */
  public enum PnpMethod {
    /** Multi-tag solve on the coprocessor (PhotonVision MULTI_TAG_PNP_ON_COPROCESSOR). */
    MULTI_TAG_PNP,
    /** Single-tag full PnP solve (SQP / iterative). */
    SINGLE_TAG_PNP,
    /**
     * Trigonometric / distance-trig solve — PhotonVision's {@code
     * estimatePnpDistanceTrigSolvePose}: uses known tag size and pixel measurements to recover
     * distance without a full PnP solve.
     */
    TRIG_PNP,
    /** Limelight MegaTag 1 (3-DOF on-device solve). */
    MEGATAG_1,
    /** Limelight MegaTag 2 (heading-constrained on-device solve). */
    MEGATAG_2,
    /** Visual-odometry / SLAM (e.g., QuestNav). */
    VISUAL_ODOMETRY,
    /** Method not exposed by this provider. */
    UNKNOWN,
  }

  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean hasTarget = false;
    public TargetRotation latestTargetRotation = new TargetRotation(new Rotation3d());
    public Pose3d latestTargetPose = new Pose3d();
    public double captureTime;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    public int[] tagIds = new int[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetRotation(Rotation3d rotation) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double effectiveSpan,
      PoseObservationType type,
      ProviderType provider,
      PnpMethod pnpMethod) {}

  /*
   * Returns the current observations of the robot.
   *
   * @return The current observations of the robot.
   */
  public default List<PoseObservation> getObservations() {
    return Arrays.asList(input.poseObservations);
  }

  /*
   * Returns whether the pose provider is currently active. A camera could be
   * inactive if it is not currently tracking a target, for example.
   *
   * @return Whether the pose provider is currently active.
   */
  public default boolean isConnected() {
    return input.connected;
  }

  public default double getCaptureTime() {
    return input.captureTime;
  }

  public void update();

  public default void resetPose(Pose3d initPose) {}

  public ProviderType getType();

  public default void logInput(String tableName) {
    Logger.processInputs(VisionConstants.SBTabVisionDisplay + "/Camera " + tableName, input);
  }

  public default Matrix<N3, N1> getStdDeviations(PoseObservation observation) {
    // Calculate standard deviations
    double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
    double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
    double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;
    if (observation.type() == PoseObservationType.MEGATAG_2) {
      linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
      angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
    }
    if (cameraIndex < VisionConstants.cameraStdDevFactors.length) {
      linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
      angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex];
    }

    return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
  }
}
