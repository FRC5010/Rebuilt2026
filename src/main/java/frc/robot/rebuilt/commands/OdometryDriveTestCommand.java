package frc.robot.rebuilt.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.utils.UnitFormat;
import org.littletonrobotics.junction.Logger;

/**
 * Drives the robot straight forward exactly 10 feet (3.048 m) at a slow, constant speed, then
 * stops. Compare the tape-measure distance to the odometry-reported distance to validate scale.
 */
public class OdometryDriveTestCommand extends Command {

  private static final double TARGET_DISTANCE_M = 3.048; // 10 ft
  private static final double DRIVE_SPEED_MPS = 0.5;
  private static final String PREFIX = "OdometryDriveTest/";

  private final GenericDrivetrain drivetrain;
  private Pose2d startPose;

  public OdometryDriveTestCommand(GenericDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    startPose = drivetrain.getPoseEstimator().getCurrentPose();
    Logger.recordOutput(PREFIX + "StartPose", startPose);
    Logger.recordOutput(PREFIX + "TargetDistanceM", TARGET_DISTANCE_M);
  }

  @Override
  public void execute() {
    drivetrain.drive(new ChassisSpeeds(DRIVE_SPEED_MPS, 0, 0));

    double traveled = traveledDistance();
    Logger.recordOutput(PREFIX + "TraveledDistanceM", traveled);
    Logger.recordOutput(PREFIX + "TraveledDistanceFt", traveled / 0.3048);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds());

    Pose2d endPose = drivetrain.getPoseEstimator().getCurrentPose();
    double measured = traveledDistance(endPose);
    double errorM = measured - TARGET_DISTANCE_M;
    double errorPct = (errorM / TARGET_DISTANCE_M) * 100.0;

    Logger.recordOutput(PREFIX + "EndPose", endPose);
    Logger.recordOutput(PREFIX + "MeasuredDistanceM", measured);
    Logger.recordOutput(PREFIX + "ErrorM", errorM);
    Logger.recordOutput(PREFIX + "ErrorPercent", errorPct);

    System.out.printf(
        "[OdometryDriveTest] Target: 10 ft 0 in | Measured: %s (%.3f m) | Error: %+.3f m (%+.1f%%)%n",
        UnitFormat.metersToFeetAndFractionalInches(measured), measured, errorM, errorPct);
  }

  @Override
  public boolean isFinished() {
    return traveledDistance() >= TARGET_DISTANCE_M;
  }

  private double traveledDistance() {
    return traveledDistance(drivetrain.getPoseEstimator().getCurrentPose());
  }

  private double traveledDistance(Pose2d current) {
    return current.getTranslation().getDistance(startPose.getTranslation());
  }
}
