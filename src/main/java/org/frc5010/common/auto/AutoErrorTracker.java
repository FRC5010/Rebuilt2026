// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoErrorTracker extends Command {
  private double errorSum = 0;
  private double minError = Double.POSITIVE_INFINITY;
  private double maxError = Double.NEGATIVE_INFINITY;

  /** Creates a new AutoErrorTracker. */
  public AutoErrorTracker() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private void logXYTarget(Pose2d pose) {
    Pose2d currentPose = AutoBuilder.getCurrentPose();
    double error = currentPose.getTranslation().getDistance(pose.getTranslation());
    Logger.recordOutput("Auto/Auto Error", error);

    if (error < minError) {
      minError = error;
    }
    if (error > maxError) {
      maxError = error;
    }

    errorSum += error * 0.02;
    Logger.recordOutput("Auto/Auto Error Sum", errorSum);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerLogging.setLogTargetPoseCallback(this::logXYTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PathPlannerLogging.setLogTargetPoseCallback(null);

    Logger.recordOutput("Auto/Auto Error Min", minError);
    Logger.recordOutput("Auto/Auto Error Max", maxError);
    Logger.recordOutput("Auto/Auto Error Sum", errorSum);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
