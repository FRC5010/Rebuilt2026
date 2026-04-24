package org.frc5010.common.constants;

import com.pathplanner.lib.config.PIDConstants;

/** A class for library constants */
public class Constants {

  public static final double loopPeriodSecs = 0.02;

  /** Auton constants */
  public static final class AutonConstants {
    /** Translation PID constants */
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 0, .1);
    /** Rotation PID constants */
    public static final PIDConstants ANGLE_PID = new PIDConstants(5, 0, .1);
    /** Freeze path progression once translational error gets large enough to indicate a block. */
    public static final double PATH_PAUSE_ERROR_METERS = 0.60;
    /** Resume trajectory time only after the robot has substantially recovered. */
    public static final double PATH_RESUME_ERROR_METERS = 0.30;
  }

  public static final class Simulation {
    public static boolean loadSimulatedField = true;
    public static String gamePieceA = "GPA";
    public static String gamePieceB = "GPB";
    public static final String AUTO_DISTURBANCE_DASHBOARD_KEY = "Auto/SimDisturbanceEnabled";
    public static final double AUTO_DISTURBANCE_START_SECONDS = 1.50;
    public static final double AUTO_DISTURBANCE_DURATION_SECONDS = 1.25;
    public static final double AUTO_DISTURBANCE_LINEAR_SCALE = 0.18;
    public static final double AUTO_DISTURBANCE_ANGULAR_SCALE = 0.40;
  }
}
