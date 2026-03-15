// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.frc5010.common.config.UnitsParser;

/** A Unit aware 3D pose JSON object */
public class Pose3dJson extends Pose2dJson
{
    public UnitValueJson z     = new UnitValueJson(0, "m");
    public UnitValueJson roll  = new UnitValueJson(0, "deg");
    public UnitValueJson pitch = new UnitValueJson(0, "deg");

    // Note: yaw is inherited from Pose2dJson as "rotation"
    public Pose3d getPose3d()
    {
        return new Pose3d(UnitsParser.parseDistance(x).in(Meters),
                          UnitsParser.parseDistance(y).in(Meters),
                          UnitsParser.parseDistance(z).in(Meters),
                          new Rotation3d(UnitsParser.parseAngle(roll).in(Radians),
                                         UnitsParser.parseAngle(pitch).in(Radians),
                                         UnitsParser.parseAngle(rotation).in(Radians)));
    }
}
