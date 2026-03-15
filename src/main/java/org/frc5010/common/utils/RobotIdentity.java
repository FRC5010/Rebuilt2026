// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/** Add your docs here. */
public class RobotIdentity
{
    public String robotClass   = "default";
    public String id           = "default";
    public boolean competition = false;
    public boolean simulate    = false;

    public static String MAC_Address = "MAC_Address";

    public static String whereAmI()
    {
        try
        {
            NetworkInterface myNI =
                NetworkInterface.networkInterfaces()
                    .filter(it -> {
                        try
                        {
                            byte[] MA = it.getHardwareAddress();
                            return null != MA;
                        }
                        catch (SocketException e)
                        {
                            throw new RuntimeException(e);
                        }
                    })
                    .findFirst()
                    .orElse(NetworkInterface.networkInterfaces().findFirst().get());
            byte[] MAC_ADDRESS       = myNI.getHardwareAddress();
            final List<Byte> macList = new ArrayList<>();
            if (null != MAC_ADDRESS)
            {
                for (byte b : MAC_ADDRESS)
                {
                    macList.add(b);
                }
            }
            String whichRobot = macList.stream()
                                    .map(it -> String.format("%02X", it))
                                    .collect(Collectors.joining(":"));
            MAC_Address = whichRobot;
            SmartDashboard.putString("My ID", whichRobot);
            return whichRobot;
        }
        catch (SocketException e)
        {
            throw new RuntimeException(e);
        }
    }
}
