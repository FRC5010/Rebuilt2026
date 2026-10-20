// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Default simulation mode. REPLAY mode is auto-selected when the JVM is launched with {@code
   * -Dlog=<path>} (Gradle {@code -Plog=<path>}) or with the {@code AKIT_LOG_PATH} environment
   * variable set — AdvantageScope's "Spawn Replay" sets the latter, so leave this on {@link
   * Mode#SIM} unless you specifically want REPLAY by default.
   */
  public static final Mode SIM_MODE = Mode.SIM;

  public static final Mode CURRENT_MODE =
      RobotBase.isReal()
          ? Mode.REAL
          : (System.getProperty("log") != null || System.getenv("AKIT_LOG_PATH") != null)
              ? Mode.REPLAY
              : SIM_MODE;

  /**
   * When false, the library skips its verbose SmartDashboard/Shuffleboard widgets and the driver
   * dashboard shows only what {@code DriverDashboard} publishes. Set to true when tuning on the
   * practice field and you want the full set of library widgets back. Debug values are always
   * available through AdvantageKit under {@code /AdvantageKit}.
   */
  public static final boolean VERBOSE_DASHBOARD = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }
}
