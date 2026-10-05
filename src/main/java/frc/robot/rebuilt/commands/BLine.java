package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.Path;
import org.frc5010.common.drive.GenericDrivetrain;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class BLine {
  public GenericDrivetrain drivetrain;
  public LoggedDashboardChooser<Command> chooser;

  public BLine(GenericDrivetrain drivetrain, LoggedDashboardChooser<Command> chooser) {
    this.drivetrain = drivetrain;
    this.chooser = chooser;
  }

  public void addAutoCommands() {
    chooser.addOption("B-Line Right 2056 Double HP", buildAuto("Right_2056_Double_HP.bline"));
    chooser.addOption("B-Line Quals 73", buildAuto("Quals_73.bline"));
    chooser.addOption("B-Line Left 2056 Double HP", buildAuto("Left_2056_Double_HP.bline"));
  }

  private Command buildAuto(String pathName) {
    Path path = new Path(pathName);
    Command auto = drivetrain.getPathBuilder().withPoseReset(drivetrain::resetPose).build(path);
    // Builder options persist — clear the pose reset before building the next path.
    drivetrain.getPathBuilder().withPoseReset(ignored -> {});
    return auto;
  }
}
