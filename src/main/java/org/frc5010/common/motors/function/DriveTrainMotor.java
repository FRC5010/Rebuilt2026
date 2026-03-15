// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors.function;

import org.frc5010.common.motors.GenericMotorController;

/** A class for handling drive train motors */
public class DriveTrainMotor extends GenericFunctionalMotor
{
    /**
     * Creates a new DriveTrainMotor
     *
     * @param motor The motor to use
     */
    public DriveTrainMotor(GenericMotorController motor, String visualName)
    {
        super(motor, visualName);
    }

    /**
     * Creates a new DriveTrainMotor
     *
     * @param motor The motor to use
     * @param slewRate The slew rate
     */
    public DriveTrainMotor(GenericMotorController motor, String visualName, double slewRate)
    {
        super(motor, visualName);
        _motor.setSlewRate(slewRate);
    }
}
