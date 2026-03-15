// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import org.frc5010.common.subsystems.LEDStrip;

/** Add your docs here. */
public class LEDStripConfigJson
{
    public int length  = 0;
    public int dataPin = 0;

    public static class SegmentConfigJson
    {
        public String name = "";
        public int start   = 0;
        public int end     = 0;
        public int order   = 1;
    }

    public SegmentConfigJson[] segments = new SegmentConfigJson[0];

    public void configure()
    {
        LEDStrip.createInstance(dataPin, length);
        for (SegmentConfigJson entry : segments)
        {
            LEDStrip.addSegment(entry.name, entry.start, entry.end, entry.order);
        }
    }
}
