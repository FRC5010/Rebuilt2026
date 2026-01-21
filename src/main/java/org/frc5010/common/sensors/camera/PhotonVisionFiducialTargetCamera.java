// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

/** A camera using the PhotonVision library. */
public class PhotonVisionFiducialTargetCamera extends PhotonVisionPoseCamera {
  /** The current list of fiducial IDs */
  protected List<Integer> targetFiducialIds = new ArrayList<>();

  /**
   * Constructor
   *
   * @param name - the name of the camera
   * @param colIndex - the column index for the dashboard
   * @param fieldLayout - the field layout
   * @param cameraToRobot - the camera-to-robot transform
   * @param fiducialIds - the list of fiducial IDs
   */
  public PhotonVisionFiducialTargetCamera(
      String name,
      int colIndex,
      AprilTagFieldLayout fieldLayout,
      Transform3d cameraToRobot,
      Supplier<Pose2d> poseSupplier,
      List<Integer> fiducialIds) {
    super(name, colIndex, fieldLayout, cameraToRobot, poseSupplier, fiducialIds);
    this.fieldLayout = fieldLayout;
    targetFiducialIds = fiducialIds;
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    super.updateCameraInfo();
    if (camResult.hasTargets()) {
      target =
          camResult.getTargets().stream()
              .filter(it -> targetFiducialIds.contains(it.getFiducialId()))
              .findFirst();

      input.hasTarget = target.isPresent();
      input.latestTargetRotation =
          target
              .map(it -> new TargetRotation(new Rotation3d(0, it.getPitch(), it.getYaw())))
              .orElse(new TargetRotation(new Rotation3d()));
      input.latestTargetPose =
          target
              .map(
                  it -> {
                    Transform3d targetTrans = robotToCamera.plus(it.getBestCameraToTarget());
                    return new Pose3d(targetTrans.getTranslation(), targetTrans.getRotation());
                  })
              .orElse(new Pose3d());
    }
  }

  /**
   * Gets the current list of fiducial IDs for this camera.
   *
   * @return the current list of fiducial IDs
   */
  public List<Integer> getFiducialIds() {
    return targetFiducialIds;
  }

  /**
   * Sets the list of fiducial IDs for this camera. The camera will only consider targets with IDs
   * in this list when locating targets. This does not change the list spefified at construction
   * time to the pose camera so that the camera can be both a pose and target camera.
   *
   * @param fiducialIds the list of fiducial IDs to consider
   */
  public void setFiducialIds(List<Integer> fiducialIds) {
    targetFiducialIds = fiducialIds;
  }

  /**
   * Updates the target information for this camera with the information from the given fiducial ID.
   * This can be used to update the camera's target information when the camera is not able to
   * locate a target.
   *
   * @param fiducialId the fiducial ID to update the target information from
   */
  public void updateTargetInfoToFiducial(int fiducialId) {
    Pose3d fiducialPose = fieldLayout.getTagPose(fiducialId).orElse(new Pose3d());
    input.latestTargetPose = fiducialPose;
    input.latestTargetRotation =
        new TargetRotation(
            new Rotation3d(
                0, fiducialPose.getRotation().getY(), fiducialPose.getRotation().getZ()));
  }

  /**
   * Gets the distance from the current robot position to the current target position.
   *
   * @param targetHeight the height of the target
   * @return the distance from the robot to the target
   */
  @Override
  public double getDistanceToTarget(double targetHeight) {
    return input
        .latestTargetPose
        .getTranslation()
        .toTranslation2d()
        .getDistance(poseSupplier.get().getTranslation());
  }
}
