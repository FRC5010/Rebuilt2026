package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import frc.robot.rebuilt.subsystems.Launcher.ShotCalculator;
import frc.robot.rebuilt.subsystems.Launcher.ShotCalculator.ShootingParameters;
import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.utils.geometry.AllianceFlipUtil;

/**
 * Interactive shot table tuning command that allows free driving while the turret tracks a target.
 *
 * <p>Unlike {@link ShotCalibrationCommand}, this command does NOT require the drivetrain - the
 * driver retains full control. The turret automatically tracks the selected target (hub or shuttle
 * tuning point) using the full aiming solver with feedforward, while hood angle and flywheel speed
 * are controlled by the operator via SmartDashboard.
 *
 * <p>Logged data points are printed to the console in a copy-paste format matching the {@link
 * ShotCalculator#createDefaultTables()} / {@link ShotCalculator#createShuttleTables()} structure.
 */
public class ShotTableTuningCommand extends Command {

  public enum TuningMode {
    HUB,
    SHUTTLE
  }

  private static final String PREFIX = "ShotTuning/";

  /** Half ball diameter in inches (5.96" / 2). */
  private static final double HALF_BALL_INCHES = 2.98;

  /**
   * Shuttle tuning target in blue-alliance coordinates: half a ball's length from the alliance wall
   * (X=0) at center field Y. Alliance-flipped automatically by {@link
   * ShotCalculator#getParameters}.
   */
  private static final Translation2d SHUTTLE_TUNE_TARGET_BLUE =
      new Translation2d(Units.inchesToMeters(HALF_BALL_INCHES), Units.inchesToMeters(12.0));

  private final Launcher launcher;
  private final ShotCalculator shotCalculator;

  private TuningMode currentMode = TuningMode.SHUTTLE;
  private final List<String> loggedHoodEntries = new ArrayList<>();
  private final List<String> loggedFlywheelEntries = new ArrayList<>();
  private final List<String> loggedTofEntries = new ArrayList<>();
  private int pointCount = 0;

  public ShotTableTuningCommand(Launcher launcher) {
    this.launcher = launcher;
    this.shotCalculator = ShotCalculator.getInstance();
    addRequirements(launcher);
  }

  @Override
  public void initialize() {
    currentMode = TuningMode.SHUTTLE;
    loggedHoodEntries.clear();
    loggedFlywheelEntries.clear();
    loggedTofEntries.clear();
    pointCount = 0;

    SmartDashboard.putBoolean(PREFIX + "Force Firing", false);
    SmartDashboard.putBoolean(PREFIX + "Log Point", false);
    SmartDashboard.putBoolean(PREFIX + "Add To Live Table", false);

    // Output telemetry
    SmartDashboard.putNumber(PREFIX + "Geometric Distance (m)", 0.0);
    SmartDashboard.putNumber(PREFIX + "Virtual Distance (m)", 0.0);
    SmartDashboard.putNumber(PREFIX + "Actual Hood (deg)", 0.0);
    SmartDashboard.putNumber(PREFIX + "Actual RPM", 0.0);
    SmartDashboard.putNumber(PREFIX + "Desired Turret (deg)", 0.0);
    SmartDashboard.putNumber(PREFIX + "Actual Turret (deg)", 0.0);
    SmartDashboard.putBoolean(PREFIX + "Hood At Goal", false);
    SmartDashboard.putBoolean(PREFIX + "Flywheel At Goal", false);
    SmartDashboard.putBoolean(PREFIX + "Turret At Goal", false);
    SmartDashboard.putBoolean(PREFIX + "Is Valid Shot", false);
    SmartDashboard.putNumber(PREFIX + "Table Guess Hood", 0.0);
    SmartDashboard.putNumber(PREFIX + "Table Guess RPM", 0.0);
    SmartDashboard.putNumber(PREFIX + "Point Count", 0);

    System.out.println("[ShotTuning] Interactive shot table tuning started.");
  }

  @Override
  public void execute() {
    // --- Step 1: Read mode and force correct shot tables ---

    switch (currentMode) {
      case HUB -> shotCalculator.setShotTables(ShotCalculator.HUB_TABLES);
      case SHUTTLE -> shotCalculator.setShotTables(ShotCalculator.SHUTTLE_TABLES);
    }

    // --- Step 2: Determine target ---
    Translation2d target =
        (currentMode == TuningMode.HUB)
            ? FieldConstants.Hub.topCenterPoint.toTranslation2d()
            : SHUTTLE_TUNE_TARGET_BLUE;

    // --- Step 3: Get shooting parameters from the solver ---
    ShootingParameters params =
        launcher.getShootingParameters(
            () -> Rebuilt.drivetrain.getPoseEstimator().getCurrentPose(), () -> target);

    // --- Step 4: Compute geometric distance ---
    Translation2d flippedTarget = AllianceFlipUtil.apply(target);
    Pose2d robotPose = Rebuilt.drivetrain.getPoseEstimator().getCurrentPose();
    double geometricDistance = robotPose.getTranslation().getDistance(flippedTarget);
    double virtualDistance =
        (params != null) ? params.distanceToVirtualTarget().in(Meters) : geometricDistance;

    // --- Step 5: Read user test values ---
    double testHoodDeg = SmartDashboard.getNumber(PREFIX + "Test Hood Angle (deg)", 35.0);
    double testFlywheelRPM = SmartDashboard.getNumber(PREFIX + "Test Flywheel RPM", 100.0);

    // --- Step 6: Apply turret tracking with user overrides ---
    // Pass params so the turret tracks the actual tuning target (shuttle or hub), not whatever
    // updateInputs last computed from the field-region auto-detection (which would always be hub).
    launcher.trackWithOverrides(params, Degrees.of(testHoodDeg), RPM.of(testFlywheelRPM));

    // --- Step 7: Update telemetry ---
    SmartDashboard.putNumber(PREFIX + "Geometric Distance (m)", geometricDistance);
    SmartDashboard.putNumber(PREFIX + "Virtual Distance (m)", virtualDistance);
    SmartDashboard.putNumber(
        PREFIX + "Actual Hood (deg)", launcher.getHoodAngleActual().in(Degrees));
    SmartDashboard.putNumber(PREFIX + "Actual RPM", launcher.getFlywheelSpeedActual().in(RPM));
    SmartDashboard.putNumber(
        PREFIX + "Desired Turret (deg)",
        (params != null) ? params.turretAngle().getDegrees() : 0.0);
    SmartDashboard.putNumber(
        PREFIX + "Actual Turret (deg)", launcher.getTurretAngleActual().in(Degrees));
    SmartDashboard.putBoolean(PREFIX + "Hood At Goal", launcher.isHoodAtGoal());
    SmartDashboard.putBoolean(PREFIX + "Flywheel At Goal", launcher.isFlywheelAtGoal());
    SmartDashboard.putBoolean(PREFIX + "Turret At Goal", launcher.isTurretAtGoal());
    SmartDashboard.putBoolean(PREFIX + "Is Valid Shot", (params != null) && params.isValid());
    SmartDashboard.putNumber(
        PREFIX + "Table Guess Hood", shotCalculator.getLookupHoodAngleDegrees(geometricDistance));
    SmartDashboard.putNumber(
        PREFIX + "Table Guess RPM", shotCalculator.getLookupFlywheelSpeed(geometricDistance));
    SmartDashboard.putNumber(PREFIX + "Point Count", pointCount);

    // --- Step 8: Handle Log Point button ---
    if (SmartDashboard.getBoolean(PREFIX + "Log Point", false)) {
      SmartDashboard.putBoolean(PREFIX + "Log Point", false);
      logDataPoint(geometricDistance, testHoodDeg, testFlywheelRPM, false);
    }

    // --- Step 9: Handle Add To Live Table button ---
    if (SmartDashboard.getBoolean(PREFIX + "Add To Live Table", false)) {
      SmartDashboard.putBoolean(PREFIX + "Add To Live Table", false);
      logDataPoint(geometricDistance, testHoodDeg, testFlywheelRPM, true);
    }
  }

  private void logDataPoint(
      double distanceMeters, double hoodDeg, double flywheelRPM, boolean addToLive) {
    pointCount++;
    double estimatedToF = distanceMeters / 15.0;

    // Format distance to enough precision
    String distStr = String.format("%.4f", distanceMeters);

    // Store entries in copy-paste format
    String hoodEntry =
        String.format("            Map.entry(%s, Rotation2d.fromDegrees(%.1f)),", distStr, hoodDeg);
    String flywheelEntry = String.format("            Map.entry(%s, %.1f),", distStr, flywheelRPM);
    String tofEntry = String.format("            Map.entry(%s, %.2f),", distStr, estimatedToF);

    loggedHoodEntries.add(hoodEntry);
    loggedFlywheelEntries.add(flywheelEntry);
    loggedTofEntries.add(tofEntry);

    // Print immediately
    System.out.println(
        String.format(
            "[ShotTuning] POINT #%d (%s) @ %sm | Hood: %.1f deg | RPM: %.1f",
            pointCount, currentMode, distStr, hoodDeg, flywheelRPM));
    System.out.println("  " + hoodEntry.trim());
    System.out.println("  " + flywheelEntry.trim());
    System.out.println("  " + tofEntry.trim());

    // Inject into live table if requested
    if (addToLive) {
      shotCalculator.addDataPoint(distanceMeters, hoodDeg, flywheelRPM, estimatedToF);
      System.out.println("  -> Added to live table");
    }
  }

  @Override
  public void end(boolean interrupted) {
    launcher.stopAllMotors();

    if (!loggedHoodEntries.isEmpty()) {
      System.out.println("\n========================================");
      System.out.println("  SHOT TABLE TUNING LOG (" + currentMode + ")");
      System.out.println("========================================");
      System.out.println("// Hood Angles (paste into hoodAngles Map.ofEntries):");
      loggedHoodEntries.forEach(System.out::println);
      System.out.println("\n// Flywheel Speeds (paste into flywheelSpeeds Map.ofEntries):");
      loggedFlywheelEntries.forEach(System.out::println);
      System.out.println("\n// Time of Flight (paste into timeOfFlightSeconds Map.ofEntries):");
      loggedTofEntries.forEach(System.out::println);
      System.out.println("========================================\n");
    }

    System.out.println(
        "[ShotTuning] Tuning ended. " + pointCount + " points logged. Interrupted: " + interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Creates the tuning command composed with an indexer force-feed so the operator can fire test
   * shots during tuning.
   */
  public static Command createWithFeed(Launcher launcher) {
    return Commands.parallel(
        new ShotTableTuningCommand(launcher),
        Commands.either(
            IndexerCommands.shouldForceCommand(),
            IndexerCommands.shouldChurnCommand(),
            () -> SmartDashboard.getBoolean(PREFIX + "Force Firing", false)));
  }
}
