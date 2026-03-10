// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import static edu.wpi.first.units.Units.Meters;

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
import org.frc5010.common.drive.pose.PoseProvider.AprilTagData;
import org.frc5010.common.drive.pose.PoseProvider.PnpMethod;
import org.frc5010.common.drive.pose.PoseProvider.PoseObservation;
import org.frc5010.common.drive.pose.PoseProvider.PoseObservationType;
import org.frc5010.common.drive.pose.PoseProvider.ProviderType;
import org.frc5010.common.vision.VisionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
        List<PhotonTrackedTarget> usedTargets = estimatedRobotPose.targetsUsed;
        int tagCount = usedTargets.size();

        double totalTagDistance = 0.0;
        double minTiltRadians = Double.MAX_VALUE; // Start arbitrarily high

        for (var target : usedTargets) {
          totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();

          var rot = target.bestCameraToTarget.getRotation();
          double yaw = rot.getZ();
          double pitch = rot.getY();
          double tilt = Math.acos(Math.cos(yaw) * Math.cos(pitch));

          if (tilt < minTiltRadians) {
            minTiltRadians = tilt;
          }

          tagIds.add((short) target.getFiducialId());
        }

        double averageDistance = (tagCount > 0) ? totalTagDistance / tagCount : 0.0;
        if (minTiltRadians == Double.MAX_VALUE) minTiltRadians = 0.0;

        SmartDashboard.putNumber("Camera/" + name() + "/Total Distance To Tag", totalTagDistance);
        SmartDashboard.putNumber(
            "Camera/" + name() + "/Photon Ambiguity", iCamResult.getBestTarget().poseAmbiguity);
        SmartDashboard.putNumberArray(
            "Camera/" + name() + "/POSE",
            new double[] {
              robotPose.getX(),
              robotPose.getY(),
              robotPose.getRotation().toRotation2d().getDegrees()
            });

        double effectiveSpan = VisionConstants.aprilTagSideLength * Math.sqrt(2.0);
        if (tagCount > 1) {
          double maxDist = 0.0;
          for (int j = 0; j < tagCount; j++) {
            for (int k = j + 1; k < tagCount; k++) {
              double dist =
                  usedTargets
                      .get(j)
                      .bestCameraToTarget
                      .getTranslation()
                      .getDistance(usedTargets.get(k).bestCameraToTarget.getTranslation());
              if (dist > maxDist) maxDist = dist;
            }
          }
          effectiveSpan = maxDist + VisionConstants.aprilTagSideLength;
        }

        observations.add(
            PoseObservation.ofAprilTag(
                iCamResult.getTimestampSeconds(),
                robotPose,
                ProviderType.FIELD_BASED,
                new AprilTagData(
                    iCamResult.getBestTarget().poseAmbiguity,
                    tagCount,
                    averageDistance,
                    effectiveSpan,
                    pnpMethod,
                    minTiltRadians,
                    PoseObservationType.PHOTONVISION)));
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
    // Non-AprilTag observations get maximum uncertainty.
    if (observation.aprilTagData().isEmpty()) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    AprilTagData aprilTagData = observation.aprilTagData().get();

    double meanReprojectionError = getMeanReprojectionError() * (1 / Math.max(Math.abs(Math.cos(aprilTagData.minTilt())), 0.01)); // sigma_p
    double fx = getFocalLengthX();
    double fy = getFocalLengthY();
    double f_avg = (fx + fy) / 2.0;

    double tagDistance = aprilTagData.averageTagDistance();
    double effectiveSpan = aprilTagData.effectiveSpan();
    int tagCount = aprilTagData.tagCount();
    double ambiguity = aprilTagData.ambiguity();

    if (tagCount == 1
        && (ambiguity > VisionConstants.maxAmbiguity
            || tagDistance > VisionConstants.maxTagDistance.in(Meters))) {
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
      double blurPixels = vt * exposureSec * (f_avg / tagDistance);
      meanReprojectionError = Math.hypot(meanReprojectionError, blurPixels);
    }

    double ambiguityPenalty = (tagCount > 1) ? 1.0 : Math.pow(1.0 / (1.0 - ambiguity), 3);

    double sigmaFieldX = 0.0;
    double sigmaFieldY = 0.0;
    double sigmaFieldTheta = 0.0;

    PnpMethod pnpMethod = aprilTagData.pnpMethod();

    if (pnpMethod == PnpMethod.MULTI_TAG_PNP) {
      double vx = Math.pow(meanReprojectionError * tagDistance / fx, 2);
      double vy = Math.pow(meanReprojectionError * tagDistance / fy, 2);
      double vz =
          Math.pow(
              (1.0 / Math.sqrt(tagCount))
                  * meanReprojectionError
                  * (tagDistance * tagDistance)
                  / (f_avg * effectiveSpan)
                  * ambiguityPenalty,
              2);

      double pitchProjY = vz * (cosPitch * cosPitch) + vy * (sinPitch * sinPitch);
      double vFieldX = vx * (sinGamma * sinGamma) + pitchProjY * (cosGamma * cosGamma);
      double vFieldY = vx * (cosGamma * cosGamma) + pitchProjY * (sinGamma * sinGamma);

      sigmaFieldX = Math.sqrt(vFieldX);
      sigmaFieldY = Math.sqrt(vFieldY);
      sigmaFieldTheta = Math.sqrt(vz) / effectiveSpan;

    } else if (pnpMethod == PnpMethod.TRIG_PNP) {

      double vd =
          Math.pow(
              meanReprojectionError
                  * (tagDistance * tagDistance)
                  / (f_avg * VisionConstants.aprilTagSideLength * Math.sqrt(2))
                  * ambiguityPenalty,
              2);
      double va = Math.pow(meanReprojectionError / fx, 2);

      double vFieldX =
          vd * (cosGamma * cosGamma) + (tagDistance * tagDistance) * va * (sinGamma * sinGamma);
      double vFieldY =
          vd * (sinGamma * sinGamma) + (tagDistance * tagDistance) * va * (cosGamma * cosGamma);

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
