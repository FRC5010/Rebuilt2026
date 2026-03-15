// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import edu.wpi.first.math.geometry.Translation3d;
import org.frc5010.common.config.UnitsParser;

/** A Unit aware 3D pose JSON object */
public class Translation3dJson extends Pose3dJson
{
    // Note: yaw is inherited from Pose2dJson as "rotation"
    public Translation3d getTranslation3d()
    {
        return new Translation3d(UnitsParser.parseDistance(x), UnitsParser.parseDistance(y),
                                 UnitsParser.parseDistance(z));
    }
}
