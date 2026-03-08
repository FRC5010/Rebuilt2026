// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.frc5010.common.vision.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** A camera using the PhotonVision library. */
public class PhotonVisionPoseCamera extends PhotonVisionCamera implements FiducialTargetCamera {
  /** The pose estimator */
  protected PhotonPoseEstimator poseEstimator;
  /** The pose supplier */
  protected Supplier<Pose2d> poseSupplier;
  /** The current list of fiducial IDs */
  protected List<Integer> fiducialIds = new ArrayList<>();

  /**
   * Optional supplier of the robot's current field-relative velocity. When provided, std devs are
   * scaled up proportionally during fast motion to trust vision less while the robot is moving.
   */
  protected Optional<Supplier<ChassisSpeeds>> robotVelocitySupplier = Optional.empty();

  private final int cameraStdDevIndex;

  /**
   * Constructor
   *
   * @param name - the name of the camera
   * @param colIndex - the column index for the dashboard
   * @param fieldLayout - the field layout
   * @param strategy - the pose strategy
   * @param cameraToRobot - the camera-to-robot transform
   * @param poseSupplier - the pose supplier
   */
  public PhotonVisionPoseCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier) {
    super(name, colIndex, cameraToRobot);
    this.poseSupplier = poseSupplier;
    this.fieldLayout = fieldLayout;
    this.cameraStdDevIndex = colIndex;

    poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraToRobot);
  }

  public PhotonVisionPoseCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier,
      List<Integer> fiducialIds) {
    super(name, colIndex, cameraToRobot);
    this.poseSupplier = poseSupplier;
    this.fieldLayout = fieldLayout;

    this.fiducialIds = fiducialIds;
    this.cameraStdDevIndex = colIndex;
    visionLayout.addDouble("Observations", () -> input.poseObservations.length);
    poseEstimator = new PhotonPoseEstimator(fieldLayout, cameraToRobot);
  }

  /**
   * Sets an optional supplier of the robot's current field-relative velocity. When provided, std
   * devs are scaled up during fast motion so vision measurements are trusted less while the robot
   * is moving quickly.
   *
   * @param velocitySupplier supplier returning a {@link ChassisSpeeds} in field-relative frame
   * @return this camera instance for fluent chaining
   */
  public PhotonVisionPoseCamera withRobotVelocitySupplier(
      Supplier<ChassisSpeeds> velocitySupplier) {
    this.robotVelocitySupplier = Optional.of(velocitySupplier);
    return this;
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    poseEstimator.addHeadingData(Timer.getFPGATimestamp(), poseSupplier.get().getRotation());

    List<PoseObservation> observations = new ArrayList<>();
    SmartDashboard.putBoolean("Camera/" + name() + "/updating", true);

    super.updateCameraInfo();
    Set<Short> tagIds = new HashSet<>();

    for (PhotonPipelineResult iCamResult : camResults) {
      SmartDashboard.putBoolean("Camera/" + name() + "/resuls", iCamResult.hasTargets());
      if (!iCamResult.hasTargets()) continue;

      Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(iCamResult);
      PnpMethod pnpMethod = PnpMethod.MULTI_TAG_PNP;

      if (estimate.isEmpty()) {
        estimate = poseEstimator.estimatePnpDistanceTrigSolvePose(iCamResult);
        pnpMethod = PnpMethod.TRIG_PNP;
      }

      if (estimate.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = estimate.get();
        Pose3d robotPose = estimatedRobotPose.estimatedPose;

        double totalTagDistance = 0.0;
        for (var iTarget : iCamResult.targets) {
          totalTagDistance += iTarget.bestCameraToTarget.getTranslation().getNorm();
        }
        // Compute the average tag distance
        int tagCount = estimatedRobotPose.targetsUsed.size();
        double averageDistance = 0.0;
        if (!iCamResult.targets.isEmpty()) {
          averageDistance = totalTagDistance / iCamResult.targets.size();
        }

        // Add tag IDs
        iCamResult.multitagResult.map(it -> tagIds.addAll(it.fiducialIDsUsed));

        SmartDashboard.putNumber(
            "Camera/" + name() + "/Total Distance To Tag " + name, totalTagDistance);
        SmartDashboard.putNumber(
            "Camera/" + name() + "/Photon Ambiguity " + name,
            iCamResult.getBestTarget().poseAmbiguity);
        SmartDashboard.putNumberArray(
            "Camera/" + name() + "/Photon Camera " + name + " POSE",
            new double[] {
              robotPose.getX(),
              robotPose.getY(),
              robotPose.getRotation().toRotation2d().getDegrees()
            });

        // Compute effective tag cluster span (S_eff)
        double effectiveSpan = VisionConstants.aprilTagSideLength * Math.sqrt(2.0);
        if (tagCount > 1) {
          double maxDist = 0.0;
          var targets = iCamResult.targets;
          for (int j = 0; j < targets.size(); j++) {
            for (int k = j + 1; k < targets.size(); k++) {
              double dist =
                  targets
                      .get(j)
                      .bestCameraToTarget
                      .getTranslation()
                      .getDistance(targets.get(k).bestCameraToTarget.getTranslation());
              if (dist > maxDist) maxDist = dist;
            }
          }
          effectiveSpan = maxDist + VisionConstants.aprilTagSideLength;
        }

        observations.add(
            new PoseObservation(
                iCamResult.getTimestampSeconds(),
                robotPose,
                iCamResult.getBestTarget().poseAmbiguity,
                tagCount,
                averageDistance,
                effectiveSpan,
                PoseObservationType.PHOTONVISION,
                ProviderType.FIELD_BASED,
                pnpMethod));
      }
    }

    input.poseObservations = new PoseObservation[observations.size()];
    for (int i = 0; i < observations.size(); i++) {
      input.poseObservations[i] = observations.get(i);
    }
    input.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      input.tagIds[i++] = id;
    }
  }

  @Override
  public Matrix<N3, N1> getStdDeviations(PoseObservation observation) {
    double meanReprojectionError = getMeanReprojectionError(); // sigma_p
    double fx = getFocalLengthX();
    double fy = getFocalLengthY();
    double f_avg = (fx + fy) / 2.0;

    double d = observation.averageTagDistance();
    double s_eff = observation.effectiveSpan();
    int n = observation.tagCount();
    double a = observation.ambiguity();

    if (n == 1 && (a > 0.15 || d > 4.0)) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    double rx = robotToCamera.getTranslation().getX();
    double ry = robotToCamera.getTranslation().getY();
    double pitch = robotToCamera.getRotation().getY();
    double mountYaw = robotToCamera.getRotation().getZ();
    double robotYaw = observation.pose().getRotation().getZ();
    double gamma = robotYaw + mountYaw; // Absolute yaw of the camera lens

    double cosGamma = Math.cos(gamma);
    double sinGamma = Math.sin(gamma);
    double cosPitch = Math.cos(pitch);
    double sinPitch = Math.sin(pitch);

    if (robotVelocitySupplier.isPresent()) {
      ChassisSpeeds speeds = robotVelocitySupplier.get().get();
      // Assuming supplier provides Field-Relative speeds
      double vxField = speeds.vxMetersPerSecond;
      double vyField = speeds.vyMetersPerSecond;
      double omega = speeds.omegaRadiansPerSecond;

      double cosRobot = Math.cos(robotYaw);
      double sinRobot = Math.sin(robotYaw);

      double vRotRobotX = -omega * ry;
      double vRotRobotY = omega * rx;

      double vRotFieldX = vRotRobotX * cosRobot - vRotRobotY * sinRobot;
      double vRotFieldY = vRotRobotX * sinRobot + vRotRobotY * cosRobot;

      double vCamX = vxField + vRotFieldX;
      double vCamY = vyField + vRotFieldY;

      double vt = Math.abs(vCamX * sinGamma - vCamY * cosGamma);

      // Apply blur to reprojection error
      double exposureSec = exposureTimeMs / 1000.0;
      double blurPixels = vt * exposureSec * (f_avg / d);
      meanReprojectionError += blurPixels;
    }

    double p = (n > 1) ? 1.0 : Math.pow(1.0 / (1.0 - a), 3);

    double sigmaFieldX = 0.0;
    double sigmaFieldY = 0.0;
    double sigmaFieldTheta = 0.0;

    PnpMethod method = observation.pnpMethod();

    if (method == PnpMethod.MULTI_TAG_PNP) {
      double vx = Math.pow(meanReprojectionError * d / fx, 2);
      double vy = Math.pow(meanReprojectionError * d / fy, 2);
      double vz =
          Math.pow((1.0 / Math.sqrt(n)) * meanReprojectionError * (d * d) / (f_avg * s_eff) * p, 2);

      double pitchProjY = vz * (cosPitch * cosPitch) + vy * (sinPitch * sinPitch);
      double vFieldX = vx * (sinGamma * sinGamma) + pitchProjY * (cosGamma * cosGamma);
      double vFieldY = vx * (cosGamma * cosGamma) + pitchProjY * (sinGamma * sinGamma);

      sigmaFieldX = Math.sqrt(vFieldX);
      sigmaFieldY = Math.sqrt(vFieldY);
      sigmaFieldTheta = Math.sqrt(vz) / s_eff;

    } else if (method == PnpMethod.TRIG_PNP) {

      double vd = Math.pow(meanReprojectionError * (d * d) / (f_avg * 0.233) * p, 2);
      double va = Math.pow(meanReprojectionError / fx, 2);

      double vFieldX = vd * (cosGamma * cosGamma) + (d * d) * va * (sinGamma * sinGamma);
      double vFieldY = vd * (sinGamma * sinGamma) + (d * d) * va * (cosGamma * cosGamma);

      sigmaFieldX = Math.sqrt(vFieldX);
      sigmaFieldY = Math.sqrt(vFieldY);
      sigmaFieldTheta = Double.MAX_VALUE;
    }

    return VecBuilder.fill(sigmaFieldX, sigmaFieldY, sigmaFieldTheta);
  }

  /**
   * Gets the current list of fiducial IDs for this camera.
   *
   * @return the current list of fiducial IDs
   */
  public List<Integer> getFiducialIds() {
    return fiducialIds;
  }

  /**
   * Sets the list of fiducial IDs for this camera. The camera will only consider targets with IDs
   * in this list when locating targets. This does not change the list spefified at construction
   * time to the pose camera so that the camera can be both a pose and target camera.
   *
   * @param fiducialIds the list of fiducial IDs to consider
   */
  public void setFiducialIds(List<Integer> fiducialIds) {
    this.fiducialIds = fiducialIds;
  }
}
