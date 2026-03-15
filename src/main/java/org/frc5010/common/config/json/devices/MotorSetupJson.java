// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import edu.wpi.first.math.Pair;
import org.frc5010.common.config.json.Translation3dJson;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.config.units.CurrentUnit;
import org.frc5010.common.motors.GenericMotorController;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.motorcontrollers.SmartMotorControllerConfig;

/** Add your docs here. */
public class MotorSetupJson
{
    public static class FollowerMotorJson
    {
        public int canId;
        public boolean inverted = false;
    }

    public String name;
    public String logLevel = "LOW";
    public String motorType;
    public String controllerType;
    public int canId;
    /** The CAN bus for the motor */
    public String canBus = "";

    public FollowerMotorJson[] followers  = new FollowerMotorJson[0];
    public String idleMode                = "BRAKE";
    public UnitValueJson currentLimit     = new UnitValueJson(40, CurrentUnit.AMPS.toString());
    public boolean inverted               = false;
    public int numberOfMotors             = 1;
    public Translation3dJson robotToMotor = new Translation3dJson();
    public String movementPlane           = "XZ";

    /**
     * Configures the followers of the given SmartMotorControllerConfig with the followers from the
     * given MotorSetupJson.
     *
     * @param motorConfig the SmartMotorControllerConfig to configure
     * @param motorSetup the MotorSetupJson from which to get the followers
     */
    public static void setupFollowers(SmartMotorControllerConfig motorConfig,
                                      MotorSetupJson motorSetup)
    {
        if (motorSetup.followers != null && motorSetup.followers.length > 0)
        {
            motorSetup.numberOfMotors += motorSetup.followers.length;
            @SuppressWarnings("unchecked")
            Pair<Object, Boolean>[] followers = new Pair[motorSetup.followers.length];
            for (int i = 0; i < motorSetup.followers.length; i++)
            {
                MotorSetupJson.FollowerMotorJson motorSetupFollower = motorSetup.followers[i];
                GenericMotorController followerMotor                = DeviceConfigReader.getMotor(
                    motorSetup.controllerType, motorSetup.motorType, motorSetupFollower.canId);
                followers[i] = new Pair<>(followerMotor.getMotor(), motorSetupFollower.inverted);
            }
            motorConfig.withFollowers(followers);
        }
    }

    public MechanismPositionConfig getMechanismPositionConfig()
    {
        return new MechanismPositionConfig()
            .withRelativePosition(robotToMotor.getTranslation3d())
            .withMovementPlane(MechanismPositionConfig.Plane.valueOf(movementPlane));
    }
}
