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
    // Competition autos
    addAuto("Bline : Right 2056 Double HP", "Right_2056_Double_HP.bline");
    addAuto("Bline : Quals 73", "Quals_73.bline");
    addAuto("Bline : Left 2056 Double HP", "Left_2056_Double_HP.bline");

    // Simple test autos
    addAuto("Test: Straight 2m", "straight-2m-2");
    addAuto("Test: Straight + Left Turn", "straight-left-turn");
    addAuto("Test: Square", "square");
    addAuto("Test: Straight + Intake", "straight-intake");
    addAuto("Test: Shoot", "shoot");
    addAuto("Test: Intake + Shoot", "Intake-Shoot");
  }

  /**
   * Adds one auto to the chooser. If its path file can't be found or loaded, it logs a warning and
   * skips it instead of crashing the whole robot program at startup.
   */
  private void addAuto(String label, String pathName) {
    try {
      chooser.addOption(label, buildAuto(pathName));
    } catch (Exception e) {
      System.err.println(
          "[BLine] Skipped auto \""
              + label
              + "\" — could not load path \""
              + pathName
              + "\": "
              + e.getMessage());
    }
  }

  private Command buildAuto(String pathName) {
    Path path = new Path(pathName);
    Command auto = drivetrain.getPathBuilder().withPoseReset(drivetrain::resetPose).build(path);
    // Builder options persist — clear the pose reset before building the next path.
    drivetrain.getPathBuilder().withPoseReset(ignored -> {});
    return auto;
  }
}
