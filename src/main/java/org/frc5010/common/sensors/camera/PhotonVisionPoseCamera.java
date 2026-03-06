// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
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
              double dist = targets.get(j).bestCameraToTarget.getTranslation()
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
                ProviderType.FIELD_BASED));
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
    // Clamp inputs to avoid degenerate values
    double d = Math.max(observation.averageTagDistance(), 0.01); // meters
    int N = Math.max(observation.tagCount(), 1);
    double sqrtN = Math.sqrt(N);
    double sEff = observation.effectiveSpan();

    // Camera intrinsics and noise constants from VisionConstants
    double f = VisionConstants.cameraFocalLength; // pixels
    double sigPx = VisionConstants.pixelNoiseStdDev; // pixels (RMS corner noise)
    double exp = VisionConstants.distanceExponent;
    double linBase = VisionConstants.linearStdDevBaseline;

    // Penalty for single-tag ambiguity
    double penalty = 1.0;
    if (N == 1) {
        double a = Math.min(observation.ambiguity(), 0.99);
        penalty = Math.pow(1.0 / (1.0 - a), 3);
    }


    double baseErr = (linBase * sigPx * Math.pow(d, exp - 1.0) / f);
    
    double stdX = baseErr * d;
    double vX = stdX * stdX;
    
    double stdZ = (1.0 / sqrtN) * baseErr * (d * d / sEff) * penalty;
    double vZ = stdZ * stdZ;
    
    double vTheta = vZ / (sEff * sEff);
    if (N == 1) {
        vTheta = 1e6; 
    }

   
    double factor = (cameraStdDevIndex < VisionConstants.cameraStdDevFactors.length)
            ? VisionConstants.cameraStdDevFactors[cameraStdDevIndex] : 1.0;
    double factorSq = factor * factor;
    
    vX = Math.min(vX * factorSq, 2500.0);
    vZ = Math.min(vZ * factorSq, 2500.0);
    if (N >= 2) {
        vTheta = Math.min(vTheta * factorSq, 2500.0);
    }

   
    double robotHeading = observation.pose().getRotation().getZ();
    double camYaw = robotToCamera.getRotation().getZ();
    double gamma = robotHeading + camYaw;
    
    double cosG = Math.cos(gamma);
    double sinG = Math.sin(gamma);
    

    double sigmaFieldX = Math.sqrt(vZ * (cosG * cosG) + vX * (sinG * sinG));
    double sigmaFieldY = Math.sqrt(vZ * (sinG * sinG) + vX * (cosG * cosG));
    double sigmaFieldTheta = Math.sqrt(vTheta);

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
