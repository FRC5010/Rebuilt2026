// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.json.RobotsJson;

/** RobotsParser is used to parse JSON configuration files to build a robot. */
public class RobotsParser
{
    /** The robot */
    GenericRobot robot;

    /** Creates the RobotsParser and reads in the robots.json file. */
    public RobotsParser()
    {
        File robotsJson = new File(Filesystem.getDeployDirectory(), "robots.json");
        if (robotsJson.exists())
        {
            // Read in the robots.json file
            try
            {
                RobotsJson configuredRobots =
                    new ObjectMapper().readValue(robotsJson, RobotsJson.class);
                robot = configuredRobots.createRobot();
            }
            catch (IOException e)
            {
                System.err.println("Error reading robots.json file: " + e.getMessage());
                e.printStackTrace();
                throw new RuntimeException(e);
            }
        }
    }

    /**
     * Returns the robot that was configured by this parser.
     *
     * @return the configured robot
     */
    public GenericRobot getRobot()
    {
        return robot;
    }
}
