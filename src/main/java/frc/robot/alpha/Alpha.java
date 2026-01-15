package frc.robot.alpha;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.example.DisplayValueSubsystem;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

public class Alpha extends GenericRobot {
  SwerveConstants swerveConstants;
  GenericDrivetrain drivetrain;
  DisplayValueSubsystem displayValueSubsystem = new DisplayValueSubsystem();

  public Alpha(String directory) {
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
}
