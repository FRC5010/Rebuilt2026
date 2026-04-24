package org.frc5010.common.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.lang.reflect.Field;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Freezes PathPlanner trajectory time when translational error gets too large so the setpoint does
 * not run away from a blocked robot during autonomous.
 */
public class AutoPausedFollowPathCommand extends FollowPathCommand {
  private static final Field AUTO_BUILDER_GLOBALS_FIELD =
      getAccessibleField(AutoBuilder.class, "globals");
  private static final Field PATH_FOLLOWING_COMMAND_BUILDER_FIELD =
      getAccessibleField(AUTO_BUILDER_GLOBALS_FIELD.getType(), "pathFollowingCommandBuilder");
  private static final Field TIMER_FIELD = getAccessibleField(FollowPathCommand.class, "timer");
  private static final Field TRAJECTORY_FIELD =
      getAccessibleField(FollowPathCommand.class, "trajectory");

  private final Supplier<Pose2d> poseSupplier;
  private final double pauseThresholdMeters;
  private final double resumeThresholdMeters;

  private boolean paused = false;
  private double pauseWallClockStartSec = Double.NaN;
  private double currentPathErrorMeters = 0.0;
  private Pose2d targetPose = new Pose2d();

  public AutoPausedFollowPathCommand(
      PathPlannerPath path,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      double pauseThresholdMeters,
      double resumeThresholdMeters,
      Subsystem... requirements) {
    super(
        path,
        poseSupplier,
        speedsSupplier,
        output,
        controller,
        robotConfig,
        shouldFlipPath,
        requirements);
    this.poseSupplier = poseSupplier;
    this.pauseThresholdMeters = pauseThresholdMeters;
    this.resumeThresholdMeters = resumeThresholdMeters;
  }

  public static void installIntoAutoBuilder(
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> speedsSupplier,
      BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
      PathFollowingController controller,
      RobotConfig robotConfig,
      BooleanSupplier shouldFlipPath,
      double pauseThresholdMeters,
      double resumeThresholdMeters,
      Subsystem... requirements) {
    try {
      Object globals = AUTO_BUILDER_GLOBALS_FIELD.get(null);
      Function<PathPlannerPath, Command> builder =
          path ->
              new AutoPausedFollowPathCommand(
                  path,
                  poseSupplier,
                  speedsSupplier,
                  output,
                  controller,
                  robotConfig,
                  shouldFlipPath,
                  pauseThresholdMeters,
                  resumeThresholdMeters,
                  requirements);
      PATH_FOLLOWING_COMMAND_BUILDER_FIELD.set(globals, builder);
    } catch (IllegalAccessException exception) {
      throw new IllegalStateException(
          "Failed to install paused PathPlanner follow command", exception);
    }
  }

  @Override
  public void initialize() {
    super.initialize();
    paused = false;
    pauseWallClockStartSec = Double.NaN;
    currentPathErrorMeters = 0.0;
    targetPose = new Pose2d();
    logState(0.0, poseSupplier.get());
  }

  @Override
  public void execute() {
    Timer timer = getTimer();
    PathPlannerTrajectory trajectory = getTrajectory();
    if (trajectory == null) {
      super.execute();
      return;
    }

    double pathTimeSeconds = timer.get();
    Pose2d currentPose = poseSupplier.get();
    PathPlannerTrajectoryState targetState = trajectory.sample(pathTimeSeconds);
    targetPose = targetState.pose;
    currentPathErrorMeters = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    if (paused) {
      if (currentPathErrorMeters <= resumeThresholdMeters) {
        paused = false;
        pauseWallClockStartSec = Double.NaN;
        timer.start();
      } else {
        timer.stop();
      }
    } else if (currentPathErrorMeters >= pauseThresholdMeters) {
      paused = true;
      pauseWallClockStartSec = Timer.getFPGATimestamp();
      timer.stop();
    }

    super.execute();
    logState(pathTimeSeconds, currentPose);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    paused = false;
    pauseWallClockStartSec = Double.NaN;
    Logger.recordOutput("Auto/Paused", false);
    Logger.recordOutput("Auto/PauseReason", interrupted ? "Interrupted" : "");
    Logger.recordOutput("Auto/PauseDurationSec", 0.0);
  }

  private void logState(double pathTimeSeconds, Pose2d currentPose) {
    Logger.recordOutput("Auto/PathError", currentPathErrorMeters);
    Logger.recordOutput("Auto/Paused", paused);
    Logger.recordOutput("Auto/PauseReason", paused ? "PathError" : "");
    Logger.recordOutput(
        "Auto/PauseDurationSec", paused ? Timer.getFPGATimestamp() - pauseWallClockStartSec : 0.0);
    Logger.recordOutput("Auto/CurrentPose", currentPose);
    Logger.recordOutput("Auto/TargetPose", targetPose);
    Logger.recordOutput("Auto/PathTimeSec", pathTimeSeconds);
  }

  private Timer getTimer() {
    try {
      return (Timer) TIMER_FIELD.get(this);
    } catch (IllegalAccessException exception) {
      throw new IllegalStateException("Failed to read FollowPathCommand timer", exception);
    }
  }

  private PathPlannerTrajectory getTrajectory() {
    try {
      return (PathPlannerTrajectory) TRAJECTORY_FIELD.get(this);
    } catch (IllegalAccessException exception) {
      throw new IllegalStateException("Failed to read FollowPathCommand trajectory", exception);
    }
  }

  private static Field getAccessibleField(Class<?> owner, String fieldName) {
    try {
      Field field = owner.getDeclaredField(fieldName);
      field.setAccessible(true);
      return field;
    } catch (NoSuchFieldException exception) {
      throw new IllegalStateException(
          "Failed to find required field '" + fieldName + "' on " + owner.getName(), exception);
    }
  }
}
