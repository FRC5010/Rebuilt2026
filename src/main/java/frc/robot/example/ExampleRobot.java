// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.example;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.frc5010.common.sensors.Controller;

/** This is an example robot class. */
public class ExampleRobot extends GenericRobot {
  SwerveConstants swerveConstants;
  GenericDrivetrain drivetrain;
  PercentControlMotor percentControlMotor;
  DisplayValueSubsystem displayValueSubsystem = new DisplayValueSubsystem();
  ExampleSubsystem exampleSubsystem;
  StateMachine stateMachine = new StateMachine("ExampleStateMachine");

  public ExampleRobot(String directory) {
    super(directory);
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    exampleSubsystem = new ExampleSubsystem();
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createAButton().onTrue(exampleSubsystem.addBallToRobot());
    driver.createBButton().onTrue(exampleSubsystem.launchBall());
    driver.createXButton().whileTrue(exampleSubsystem.setVelocity(RPM.of(2000)));
    driver.createYButton().onTrue(exampleSubsystem.sysIdShooter());
    driver.createLeftBumper().onTrue(exampleSubsystem.setDutyCycle(0));
    driver.createRightBumper().whileTrue(exampleSubsystem.setVelocity(RPM.of(-300)));

    State idle = stateMachine.addState("idle", Commands.idle());
    stateMachine.setInitialState(idle);
    State prepping = stateMachine.addState("running", exampleSubsystem.setVelocity(RPM.of(3000)));
    idle.switchTo(prepping).when(driver.RIGHT_BUMPER);

    State launching = stateMachine.addState("launching", exampleSubsystem.launchBall());
    prepping.switchTo(launching).when(() -> exampleSubsystem.getVelocity() == RPM.of(3000));
    launching.switchTo(idle).when(() -> !driver.RIGHT_BUMPER.getAsBoolean());
    prepping.switchTo(idle).when(() -> !driver.RIGHT_BUMPER.getAsBoolean());

    // exampleSubsystem.setDefaultCommand(stateMachine);

    // driver.createYButton().onTrue(exampleSubsystem.setVelocityControlMotorReference(() -> 3500))
    //     .onFalse(exampleSubsystem.setVelocityControlMotorReference(() -> 0));
    // driver.createXButton().onTrue(exampleSubsystem.setVelocityControlMotorReference(() -> 2000))
    //     .onFalse(exampleSubsystem.setVelocityControlMotorReference(() -> 0));
    // driver.createAButton().whileTrue(exampleSubsystem.setAngularMotorReference(() -> 90))
    //     .whileFalse(exampleSubsystem.setAngularMotorReference(() -> 0));
    //    driver.createBButton().whileTrue(((YAGSLSwerveDrivetrain)drivetrain).driveToPose(new
    // Pose2d(8, 4, new Rotation2d())));
    // driver.createAButton().whileTrue(exampleSubsystem.setElevatorHeight(() -> Meters.of(0.5)));
    // driver.createBButton().whileTrue(exampleSubsystem.setElevatorHeight(() -> Meters.of(2)));
    // driver
    //     .createXButton()
    //     .whileTrue(exampleSubsystem.setArmAngle(() -> Degrees.of(90)))
    //     .whileFalse(exampleSubsystem.setArmAngle(() -> Degrees.of(0)));
    // driver
    //     .createYButton()
    //     .whileTrue(exampleSubsystem.setPivotAngle(() -> Degrees.of(90)))
    //     .whileFalse(exampleSubsystem.setPivotAngle(() -> Degrees.of(0)));
    // driver.createRightBumper().whileTrue(exampleSubsystem.driveElevator(() -> 0.5));
    // driver.createLeftBumper().whileTrue(exampleSubsystem.driveElevator(() -> -0.5));
    // driver.createStartButton().whileTrue(exampleSubsystem.getElevatorSysId());
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    driver.setRightTrigger(driver.createRightTrigger());
    exampleSubsystem.setDefaultCommand(
        exampleSubsystem.getDefaultCommand(() -> operator.getLeftYAxis()));
    drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
  }

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
