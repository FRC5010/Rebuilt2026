// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.frc5010.common.sensors.Controller;

public class Intake extends GenericSubsystem {
  private PercentControlMotor Spintake;

  /** Creates a new Intake. */
  public Intake() {
    super("intake.json");
    Spintake = (PercentControlMotor) devices.get("spintake");
  }

  public void RunSpintake(double speed) {
    Spintake.set(speed);
  }

  public void ConfigController(Controller controller) {
    controller.createLeftBumper().whileTrue(spintakeCommand(.25));
  }

  public Command spintakeCommand(double speed) {
    return Commands.run(
            () -> {
              RunSpintake(.25);
            })
        .finallyDo(
            () -> {
              RunSpintake(0);
            });
  }
}
