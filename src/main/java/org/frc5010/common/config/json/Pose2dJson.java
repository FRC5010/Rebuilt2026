// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.frc5010.common.config.UnitsParser;

/** A Unit aware 2D pose JSON object */
public class Pose2dJson
{
    public UnitValueJson x        = new UnitValueJson(0, "m");
    public UnitValueJson y        = new UnitValueJson(0, "m");
    public UnitValueJson rotation = new UnitValueJson(0, "deg");

    /**
     * Get the 2D pose represented by this JSON object.
     *
     * @return the 2D pose
     */
    public Pose2d getPose2d()
    {
        return new Pose2d(UnitsParser.parseDistance(x).in(Meters),
                          UnitsParser.parseDistance(y).in(Meters),
                          new Rotation2d(UnitsParser.parseAngle(rotation).in(Radians)));
    }
}
