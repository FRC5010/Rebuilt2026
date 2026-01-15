// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import java.util.Map;
import org.frc5010.common.config.json.UnitValueJson;

/** SwerveConstantsJson class holds configuration constants for a swerve drive system. */
public class DrivetrainConstantsJson {
  /** The track width */
  public UnitValueJson trackWidth = new UnitValueJson(20, "inches");
  /** The wheel base */
  public UnitValueJson wheelBase = new UnitValueJson(20, "inches");
  /** The diameter of the wheel */
  public UnitValueJson wheelDiameter = new UnitValueJson(4, "inches");
  /** The width of the frame of the robot including bumpers */
  public UnitValueJson bumperFrameWidth = new UnitValueJson(25, "inches");
  /** The length of the frame of the robot including bumpers */
  public UnitValueJson bumperFrameLength = new UnitValueJson(25, "inches");
  /** The maximum drive speed of the robot */
  public UnitValueJson maxDriveSpeed = new UnitValueJson(5, "m/sec");
  /** The settings for the gyro */
  public GyroSettingsConfigurationJson gyro = new GyroSettingsConfigurationJson();
  /** The gear ratio of the drive motor */
  public String driveGearRatio = "1:1";
  /** The gear ratio of the steer motor */
  public String steerGearRatio = "1:1";
  /** The system ID values for the drive motor */
  public MotorSystemIdJson driveMotorControl;
  /** The system ID values for the steer motor */
  public MotorSystemIdJson steerMotorControl;
  /**
   * The module configurations, mapped by module name which should be frontLeft, frontRight,
   * backLeft, backRight
   */
  public Map<String, ModuleConfigJson> modules;
  /**
   * The ratio of the drive motor to the steer motor. Every 1 rotation of the azimuth results in
   * kCoupleRatio drive motor turns;
   */
  public double coupleRatio = 1.0;
  /** Whether the left side should be inverted */
  public boolean invertLeftSide = true;
  /** Whether the right side should be inverted */
  public boolean invertRightSide = true;
  /** The steer inertia constant */
  public UnitValueJson steerInertia = new UnitValueJson(0.001, "kg*m^2");
  /** The drive inertia constant */
  public UnitValueJson driveInertia = new UnitValueJson(0.001, "kg*m^2");
  /** The mass of the robot */
  public UnitValueJson robotMass = new UnitValueJson(50, "kg");
  /** The coefficient of friction of the wheels */
  public double wheelCOF = 1.2;
  /** The current limit of the drive motors */
  public UnitValueJson slipCurrent = new UnitValueJson(40, "amps");
  /** The CAN bus for the drivetrain */
  public String canbus = "";

  /** Configuration for a swerve module */
  public static class ModuleConfigJson {
    /** The motor setup for the drive motor */
    public MotorSetupJson driveMotorSetup = new MotorSetupJson();
    /** The motor setup for the steer motor */
    public MotorSetupJson steerMotorSetup = new MotorSetupJson();
    /** The encoder ID */
    public int encoderId = 2;
    /** The absolute encoder offset */
    public UnitValueJson absoluteOffset = new UnitValueJson(0, "degrees");
    /** Whether the encoder is inverted */
    public boolean encoderInverted = false;
  }
}
