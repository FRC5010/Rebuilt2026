// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.telemetry;

/**
 * Global switch for verbose SmartDashboard and Shuffleboard output from the library.
 *
 * <p>When verbose output is off, the library skips the dashboard widgets that WPILib polls every
 * loop: Shuffleboard tabs, {@code SmartDashboard.putData} sendables, and supplier-backed display
 * strings. Values that matter for debugging should be logged through AdvantageKit instead, which
 * still publishes them to NetworkTables under {@code /AdvantageKit}.
 *
 * <p>The setting must be applied before the robot and its subsystems are constructed, because the
 * dashboard widgets are created in constructors. Verbose output is on by default so existing robots
 * keep their current behavior.
 */
public final class DashboardSettings {
  private static boolean verbose = true;

  private DashboardSettings() {}

  /**
   * Returns whether verbose dashboard output is enabled.
   *
   * @return true if the library should publish its full set of dashboard widgets
   */
  public static boolean isVerbose() {
    return verbose;
  }

  /**
   * Enables or disables verbose dashboard output. Call this before the robot is constructed.
   *
   * @param isVerbose true to publish the full set of dashboard widgets, false to publish only what
   *     a robot explicitly puts on the dashboard
   */
  public static void setVerbose(boolean isVerbose) {
    verbose = isVerbose;
  }
}
