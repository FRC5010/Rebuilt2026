// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;
import yams.mechanisms.positional.Elevator;

public class Climb extends GenericSubsystem {
  /** Creates a new Climb. */
  private static Elevator climber;

  public Command climberCommand(double height) {
    return Commands.run(
            () -> {
              setHeight(height);
            })
        .finallyDo(
            () -> {
              setHeight(0);
            });
  }

  public void ConfigController(Controller controller) {
    controller.createBButton().whileTrue(climberCommand(.5));
  }

  public Climb() {
    super("climb.json");
    climber = (Elevator) devices.get("Climb");
  }

  public void setHeight(double height) {
    Distance mydist = Meters.of(height);
    climber.getMotorController().setPosition(mydist);
  }
}
