package frc.robot.rebuilt.commands;
import org.frc5010.common.drive.GenericDrivetrain;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.Path;

public class BLine {
 public GenericDrivetrain drivetrain;
 public LoggedDashboardChooser chooser;


 public BLine(GenericDrivetrain drivetrain, LoggedDashboardChooser chooser) {
 this.drivetrain = drivetrain;
 this.chooser = chooser;
}

 public void addAutoCommands() {
    chooser.addOption("myFirstAuto", myFirstAuto());
 }
 public Command myFirstAuto(){
Path firstStraight = new Path("firstAuto");

Command firstAuto = drivetrain.getPathBuilder()
    .withPoseReset(drivetrain::resetPose)
    .build(firstStraight);

// Builder options persist. Clear this before building any later path command.
drivetrain.getPathBuilder().withPoseReset(ignored -> {});
return firstAuto;
 }
}
