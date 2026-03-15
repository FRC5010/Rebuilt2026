// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.io.IOException;

/** Add your docs here. */
public class LEDStripParser
{
    public static void parse(String robotDirectory)
    {
        try
        {
            File directory =
                new File(Filesystem.getDeployDirectory(), robotDirectory + "/subsystems");
            DeviceConfigReader.checkDirectory(directory);
            File deviceFile = new File(directory, "led_strip.json");
            if (!deviceFile.exists())
            {
                return;
            }
            LEDStripConfigJson ledStrip =
                new ObjectMapper().readValue(deviceFile, LEDStripConfigJson.class);
            ledStrip.configure();
        }
        catch (IOException e)
        {
            System.out.println("Error reading device configuration: " + e.getMessage());
            return;
        }
    }
}
