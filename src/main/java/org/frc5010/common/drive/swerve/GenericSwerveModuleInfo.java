// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import org.frc5010.common.drive.swerve.akit.Module;
import swervelib.SwerveModule;

/** Add your docs here. */
public record GenericSwerveModuleInfo(double steerAbsoluteDegrees,
                                      double steerRelativeDegrees,
                                      double driveRelativePositionMeters,
                                      double driveVelocityMetersPerSecond,
                                      double steerVelocityDegreesPerSecond,
                                      double expectedSteerDegrees)
{
    public GenericSwerveModuleInfo(SwerveModule module)
    {
        this(module.getAbsolutePosition(), module.getRelativePosition(),
             module.getDriveMotor().getPosition(), module.getDriveMotor().getVelocity(),
             module.getAngleMotor().getVelocity(), module.getState().angle.getDegrees());
    }

    public GenericSwerveModuleInfo(Module module)
    {
        this(module.getAngle().getDegrees(), module.getAngle().getDegrees(),
             module.getPosition().distanceMeters, module.getVelocityMetersPerSec(), 0.0,
             module.getAngle().getDegrees());
    }
}
