// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.arch;

import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

/**
 * GenericMechanism should be used as the parent class of any mechanism It enforces the use of the
 * functions:
 */
public abstract class GenericMechanism implements WpiHelperInterface
{
    /** The visual representation of the mechanism */
    @AutoLogOutput protected LoggedMechanism2d mechVisual;
    /** The log prefix */
    protected String logPrefix = getClass().getSimpleName();
    /** The display values helper */
    protected final DisplayValuesHelper DashBoard;

    /** Constructor for GenericMechanism */
    public GenericMechanism()
    {
        this.mechVisual =
            new LoggedMechanism2d(RobotConstantsDef.robotVisualH, RobotConstantsDef.robotVisualV);
        DashBoard = new DisplayValuesHelper(logPrefix);
    }

    /**
     * Constructor for GenericMechanism
     *
     * @param tabName - the name of the display tab
     */
    public GenericMechanism(String tabName)
    {
        this.mechVisual =
            new LoggedMechanism2d(RobotConstantsDef.robotVisualH, RobotConstantsDef.robotVisualV);
        DashBoard = new DisplayValuesHelper(logPrefix);
    }

    /**
     * configureButtonBindings should map button/axis controls to commands
     *
     * @param driver - driver joystick
     * @param operator - operator joystick
     */
    public abstract void configureButtonBindings(Controller driver, Controller operator);

    /**
     * configureAltButtonBindings should map button/axis controls to commands
     *
     * @param driver - driver joystick
     * @param operator - operator joystick
     */
    public void configureAltButtonBindings(Controller driver, Controller operator) {}

    /**
     * setupDefaultCommands should setup the default commands needed by subsystems It could check
     * for Test mode and enable different commands
     *
     * @param driver - driver joystick
     * @param operator - operator joystick
     */
    public abstract void setupDefaultCommands(Controller driver, Controller operator);

    /**
     * Sets up the default commands for testing purposes.
     *
     * @param driver the driver controller
     * @param operator the operator controller
     */
    public void setupAltDefaultCommmands(Controller driver, Controller operator) {}

    /**
     * initRealOrSim should check the real or simulation state of the robot and initialize its code
     * accordingly
     */
    protected abstract void initRealOrSim();

    /** setupPreferences should be implemented in place of using Constants files */
    protected void setupPreferences() {}

    /** Used to initialize auto commands for the robot */
    public abstract void initAutoCommands();

    /**
     * Used to wrap the selected auto command in additional behavior
     *
     * @param autoCommand the auto command
     * @return the wrapped auto command
     */
    public abstract Command generateAutoCommand(Command autoCommand);

    /** Executed periodically when robot is disabled */
    public void disabledInit() {}

    /** Executed periodically when robot is disabled */
    public void disabledPeriodic() {}
}
