// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.subsystems.Climb;
import frc.robot.rebuilt.subsystems.Indexer;
import frc.robot.rebuilt.subsystems.Intake;
import frc.robot.rebuilt.subsystems.Launcher;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.frc5010.common.sensors.Controller;

/** This is an example robot class. */
public class Rebuilt extends GenericRobot {
  SwerveConstants swerveConstants;
  GenericDrivetrain drivetrain;
  PercentControlMotor percentControlMotor;
  StateMachine stateMachine = new StateMachine("ExampleStateMachine");
  Indexer indexer;
  Climb climb;
  Intake intake;
  Launcher launcher;

  public Rebuilt(String directory) {
    super(directory);
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    indexer = new Indexer();
    climb = new Climb();
    intake = new Intake();
    launcher = new Launcher();
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    indexer.ConfigController(driver);
    intake.ConfigController(driver);
    climb.ConfigController(driver);
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {}

  @Override
  public void initAutoCommands() {
    drivetrain.setAutoBuilder();
  }

  @Override
  public Command generateAutoCommand(Command autoCommand) {
    return drivetrain.generateAutoCommand(autoCommand);
  }

  @Override
  public void buildAutoCommands() {
    super.buildAutoCommands();
    selectableCommand.addOption("Do Nothing", Commands.none());
    drivetrain.addAutoCommands(selectableCommand);
  }
}
