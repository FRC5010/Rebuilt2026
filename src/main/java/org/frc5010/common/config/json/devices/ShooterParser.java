// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import yams.mechanisms.velocity.FlyWheel;

/** Add your docs here. */
public class ShooterParser
{
    /**
     * Reads a device configuration from the given file and adds it to the system.
     *
     * @param subDirectory the subdirectory containing the device configuration file
     * @param filename the name of the device configuration file
     * @param system the system to add the device to
     * @return the configured pivot, or null if there was an error
     */
    public static FlyWheel parse(String subDirectory, String filename, SubsystemBase system)
    {
        try
        {
            File directory = new File(Filesystem.getDeployDirectory(), subDirectory);
            DeviceConfigReader.checkDirectory(directory);
            File deviceFile = new File(directory, filename);
            YamsShooterConfigurationJson yamsShooterConfigurationJson =
                new ObjectMapper().readValue(deviceFile, YamsShooterConfigurationJson.class);
            return yamsShooterConfigurationJson.configure(system);
        }
        catch (IOException e)
        {
            System.out.println("Error reading device configuration: " + e.getMessage());
            return null;
        }
    }
}
