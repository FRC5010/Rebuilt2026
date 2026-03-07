// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.vision;

/** A place to define library vision constants */
public class VisionConstants {
  public static final double CAMERA_CAL_DISTANCE = 120;
  public static String SBTabVisionDisplay = "Vision";

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.15;
  public static double maxZError = 1;

  public static double linearStdDevBaseline = 0.05; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Camera intrinsics
  public static double cameraFocalLength = 900.0; // pixels

  // FRC 2026 AprilTag side length in meters (6.5 inches = 165.1 mm)
  public static double aprilTagSideLength = 0.1651; // meters

  public static double pixelNoiseStdDev = 0.5; // pixels

  public static double distanceExponent = 1.5;

  public static double maxLinearStdDev = 50.0; // meters
  public static double maxAngularStdDev = 50.0; // radians

  public static double[] cameraStdDevFactors = new double[] {1.0, 1.0};

  // Multipliers to apply for MegaTag 2 observations (Limelight)
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
