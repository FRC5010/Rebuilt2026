// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** A camera using the PhotonVision library. */
public class PhotonVisionCamera extends GenericCamera {
  /** The camera */
  protected PhotonCamera camera;
  /** The field layout */
  protected AprilTagFieldLayout fieldLayout;
  /** The target, if any */
  protected Optional<PhotonTrackedTarget> target = Optional.empty();
  /** The latest camera result */
  protected PhotonPipelineResult camResult;
  /** The latest camera results */
  protected List<PhotonPipelineResult> camResults;

  /** Camera exposure time in milliseconds (0 = camera default / auto-exposure). */
  protected double exposureTimeMs = 0;
  /**
   * Camera focal length along the horizontal (X) axis in pixels from calibration (0 = use global
   * default).
   */
  protected double focalLengthX = 0;
  /**
   * Camera focal length along the vertical (Y) axis in pixels from calibration (0 = use
   * focalLengthX as fallback).
   */
  protected double focalLengthY = 0;
  /** Mean pixel reprojection error from camera calibration in pixels (0 = use global default). */
  protected double meanReprojectionError = 0;

  /**
   * Constructor
   *
   * @param name - the name of the camera
   * @param colIndex - the column index for the dashboard
   * @param cameraToRobot - the camera-to-robot transform
   */
  public PhotonVisionCamera(String name, int colIndex, Transform3d cameraToRobot) {
    super(name, colIndex, cameraToRobot);
    this.robotToCamera = cameraToRobot;
    camera = new PhotonCamera(name);
  }

  /** Update the camera and target with the latest result */
  @Override
  public void updateCameraInfo() {
    camResults = camera.getAllUnreadResults();
    camResult = camResults.stream().findFirst().orElse(new PhotonPipelineResult());
    input.connected = camera.isConnected();
    input.captureTime = camResult.getTimestampSeconds();
  }

  /**
   * Get the target area
   *
   * @return the target area
   */
  @Override
  public double getTargetArea() {
    return target.map(t -> t.getArea()).orElse(Double.MAX_VALUE);
  }

  // -----------------------------------------------------------------------
  // Calibration parameter accessors
  // -----------------------------------------------------------------------

  /**
   * Returns the configured exposure time for this camera in milliseconds.
   *
   * @return exposure time in ms, or 0 if the camera default / auto-exposure should be used
   */
  public double getExposureTimeMs() {
    return exposureTimeMs;
  }

  /**
   * Returns the horizontal focal length (fx) in pixels from the camera calibration.
   *
   * @return focal length in pixels, or 0 if the global default should be used
   */
  public double getFocalLengthX() {
    return focalLengthX;
  }

  /**
   * Returns the vertical focal length (fy) in pixels from the camera calibration.
   *
   * @return focal length in pixels, or 0 if {@link #getFocalLengthX()} should be used as fallback
   */
  public double getFocalLengthY() {
    return focalLengthY;
  }

  /**
   * Returns the mean pixel reprojection error from the camera calibration.
   *
   * @return mean reprojection error in pixels, or 0 if the global default should be used
   */
  public double getMeanReprojectionError() {
    return meanReprojectionError;
  }

  /**
   * Applies all four calibration parameters from a camera JSON configuration in one call.
   *
   * @param exposureTimeMs exposure time in milliseconds (0 = camera default)
   * @param focalLengthX horizontal focal length in pixels (0 = use global default)
   * @param focalLengthY vertical focal length in pixels (0 = fall back to focalLengthX)
   * @param meanReprojectionError mean pixel reprojection error in pixels (0 = use global default)
   */
  public void setCalibrationParams(
      double exposureTimeMs,
      double focalLengthX,
      double focalLengthY,
      double meanReprojectionError) {
    this.exposureTimeMs = exposureTimeMs;
    this.focalLengthX = focalLengthX;
    this.focalLengthY = focalLengthY;
    this.meanReprojectionError = meanReprojectionError;
  }

  @Override
  public ProviderType getType() {
    return ProviderType.FIELD_BASED;
  }
}
