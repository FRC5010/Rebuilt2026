package frc.robot.tigershark;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.example.DisplayValueSubsystem;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

public class TigerShark extends GenericRobot {
  SwerveConstants swerveConstants;
  GenericDrivetrain drivetrain;
  DisplayValueSubsystem displayValueSubsystem = new DisplayValueSubsystem();

  public TigerShark(String directory) {
    super(directory);
    drivetrain = (GenericDrivetrain) getSubsystem(ConfigConstants.DRIVETRAIN);
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {}

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
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
