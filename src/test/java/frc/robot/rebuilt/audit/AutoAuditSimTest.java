package frc.robot.rebuilt.audit;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Robot;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.Rebuilt;
import frc.robot.rebuilt.commands.LauncherCommands.LauncherState;
import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import org.frc5010.common.drive.swerve.SwerveDriveFunctions;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.condition.EnabledIfEnvironmentVariable;

/**
 * Headless full-robot simulation of ONE auto, checking the hood under the trench beams.
 *
 * <p>Opt-in: skipped unless {@code AUDIT_AUTO} is set. It boots the real {@code Robot} under the
 * HAL simulator (real drivetrain physics, launcher, intake, indexer, command scheduler), selects
 * the auto on the dashboard chooser, places the robot at the auto's start pose, enables autonomous
 * and steps simulated time at 20 ms. Each auto needs a fresh JVM (the robot is a singleton), so use
 * {@code tools/auto-audit/run-sim-audit.sh} to run several.
 *
 * <p>Environment:
 *
 * <ul>
 *   <li>{@code AUDIT_KIND}: {@code bline} or {@code pathplanner}
 *   <li>{@code AUDIT_AUTO}: the auto chooser label (e.g. "Bline : Right 5010 Double" or a
 *       PathPlanner auto name)
 *   <li>{@code AUDIT_FILE}: B-Line path name (deploy/autos/paths/NAME.json) or PathPlanner auto
 *       name (used for the start pose and as the baseline key)
 *   <li>{@code AUDIT_ALLIANCE} Blue (default) or Red, {@code AUDIT_SECONDS} (default 20), {@code
 *       AUDIT_OUT} optional CSV path
 *   <li>{@code AUDIT_UPDATE_BASELINE=1} record this auto's verdict in baseline.json; {@code
 *       AUDIT_STRICT_BASELINE=1} also fail if a baseline entry no longer reproduces
 * </ul>
 *
 * <p><b>B-Line</b> sim verdicts are repeatable (worst hood angle differed by at most 0.4 deg
 * between identical runs), so they are enforced against the baseline. <b>PathPlanner</b> sim
 * verdicts are NOT repeatable: the sim drivetrain lags its trajectory (paths end on their timers
 * and markers fire on trajectory time), and the resulting hood angle under a beam differed by up to
 * 17 deg between identical runs. They are printed as a stress test but not enforced (the
 * deterministic static analysis is the PathPlanner gate); set {@code
 * AUDIT_SIM_ENFORCE_PATHPLANNER=1} to enforce anyway. Only violations above {@link
 * AuditBaseline#SIM_THRESHOLD_DEG} are tracked.
 */
public class AutoAuditSimTest {
  public static class SimRobot extends Robot {}

  private static final ObjectMapper MAPPER = new ObjectMapper();

  private static String env(String k, String d) {
    String v = System.getenv(k);
    return v == null || v.isEmpty() ? d : v;
  }

  private static Pose2d startPose(String kind, String file) throws Exception {
    if (kind.equals("bline")) {
      JsonNode first =
          MAPPER
              .readTree(new File("src/main/deploy/autos/paths/" + file + ".json"))
              .get("path_elements")
              .get(0);
      return new Pose2d(
          first.get("translation_target").get("x_meters").asDouble(),
          first.get("translation_target").get("y_meters").asDouble(),
          new Rotation2d(first.get("rotation_target").get("rotation_radians").asDouble()));
    }
    JsonNode auto =
        MAPPER.readTree(new File("src/main/deploy/pathplanner/autos/" + file + ".auto"));
    String firstPath = firstPath(auto.get("command"));
    JsonNode path =
        MAPPER.readTree(new File("src/main/deploy/pathplanner/paths/" + firstPath + ".path"));
    JsonNode a0 = path.get("waypoints").get(0).get("anchor");
    double rot =
        path.hasNonNull("idealStartingState")
            ? path.get("idealStartingState").get("rotation").asDouble()
            : 0.0;
    return new Pose2d(a0.get("x").asDouble(), a0.get("y").asDouble(), Rotation2d.fromDegrees(rot));
  }

  private static String firstPath(JsonNode c) {
    if (c.get("type").asText().equals("path")) return c.get("data").get("pathName").asText();
    if (c.get("data").has("commands")) {
      for (JsonNode k : c.get("data").get("commands")) {
        String r = firstPath(k);
        if (r != null) return r;
      }
    }
    return null;
  }

  @Test
  @EnabledIfEnvironmentVariable(named = "AUDIT_AUTO", matches = ".+")
  public void simulateAutoAndCheckHoodUnderTrench() throws Exception {
    String kind = env("AUDIT_KIND", "bline");
    String label = env("AUDIT_AUTO", null);
    String file = env("AUDIT_FILE", null);
    String alliance = env("AUDIT_ALLIANCE", "Blue");
    double seconds = Double.parseDouble(env("AUDIT_SECONDS", "20"));
    String csvOut = env("AUDIT_OUT", "");
    assertTrue(
        kind.equals("bline") || kind.equals("pathplanner"), "AUDIT_KIND must be bline|pathplanner");
    assertTrue(file != null, "AUDIT_FILE is required");

    // The robot's config parsers assert on optional directories; a real sim run has assertions off.
    ClassLoader.getSystemClassLoader().setDefaultAssertionStatus(false);

    Pose2d start = startPose(kind, file);
    boolean red = alliance.equals("Red");
    Pose2d placed = red ? org.frc5010.common.utils.geometry.AllianceFlipUtil.apply(start) : start;

    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    SimRobot robot = new SimRobot();
    final Throwable[] robotError = new Throwable[1];
    Thread robotThread = new Thread(robot::startCompetition, "sim-robot");
    robotThread.setDaemon(true);
    robotThread.setUncaughtExceptionHandler((t, e) -> robotError[0] = e);
    robotThread.start();

    long deadline = System.currentTimeMillis() + 240_000;
    while ((Rebuilt.launcher == null || Rebuilt.drivetrain == null || Rebuilt.intake == null)
        && System.currentTimeMillis() < deadline) {
      Thread.sleep(100);
    }
    Thread.sleep(4000); // let robotInit finish
    for (int i = 0; i < 50; i++) SimHooks.stepTiming(0.02);

    // Select the auto. The chooser only refreshes in disabledPeriodic, which Robot gates behind a
    // wheel-lock timer, so step until the chooser reports the selection.
    NetworkTableEntry selected = chooser("selected");
    NetworkTableEntry active = chooser("active");
    selected.setString(label);
    SwerveDriveFunctions.driveSimulation.setSimulationWorldPose(placed);
    Rebuilt.drivetrain.resetPose(placed);
    for (int i = 0; i < 1000 && !label.equals(active.getString("")); i++) SimHooks.stepTiming(0.02);
    for (int i = 0; i < 50; i++) SimHooks.stepTiming(0.02);
    assertEquals(label, active.getString(""), "auto chooser did not select the requested auto");

    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    double t0 = Timer.getFPGATimestamp();

    // Run and record (blue frame, so Red runs use the same trench geometry)
    double L = FieldConstants.fieldLength;
    double W = FieldConstants.fieldWidth;
    List<double[]> samples = new ArrayList<>(); // t, x, y, hoodDeg
    StringBuilder csv =
        new StringBuilder("t,simX,simY,simDeg,hoodDeg,launcherCur,launcherReq,intakeCur\n");
    int ticks = (int) (seconds / 0.02);
    for (int i = 0; i < ticks && robotError[0] == null; i++) {
      SimHooks.stepTiming(0.02);
      double t = Timer.getFPGATimestamp() - t0;
      Pose2d p = SwerveDriveFunctions.driveSimulation.getSimulatedDriveTrainPose();
      double x = red ? L - p.getX() : p.getX();
      double y = red ? W - p.getY() : p.getY();
      double hood = Rebuilt.launcher.getHoodAngleActual().in(Units.Degrees);
      samples.add(new double[] {t, x, y, hood});
      String req = "?";
      for (LauncherState s : LauncherState.values())
        if (Rebuilt.launcher.isRequested(s)) req = s.name();
      csv.append(
          String.format(
              "%.3f,%.3f,%.3f,%.1f,%.2f,%s,%s,%s%n",
              t,
              x,
              y,
              p.getRotation().getDegrees() + (red ? 180 : 0),
              hood,
              Rebuilt.launcher.getCurrentState(),
              req,
              Rebuilt.intake.getCurrentState()));
    }
    if (!csvOut.isEmpty()) {
      try (PrintWriter pw = new PrintWriter(csvOut)) {
        pw.print(csv);
      }
    }
    assertTrue(robotError[0] == null, "robot thread died: " + robotError[0]);

    // Stays under a trench beam (robot centre) and the hood during each
    List<String> report = new ArrayList<>();
    double worst = 0;
    String zone = null;
    double stayStart = 0, stayMax = 0;
    int pass = 0;
    for (int i = 0; i <= samples.size(); i++) {
      String z =
          i < samples.size()
              ? TrenchGeometry.zone(samples.get(i)[1], samples.get(i)[2], TrenchGeometry.CENTER)
              : null;
      if (zone != null && (z == null || !z.equals(zone))) {
        report.add(
            String.format(
                "pass %d %s t=%.1f-%.1fs max hood %.1f deg",
                pass, zone, stayStart, samples.get(i - 1)[0], stayMax));
        worst = Math.max(worst, stayMax);
        zone = null;
      }
      if (z != null) {
        double[] s = samples.get(i);
        if (zone == null) {
          zone = z;
          pass++;
          stayStart = s[0];
          stayMax = s[3];
        } else {
          stayMax = Math.max(stayMax, s[3]);
        }
      }
    }
    boolean violating = worst > AuditBaseline.SIM_THRESHOLD_DEG;
    System.out.printf(
        "[auto-audit] SIM %s '%s' (%s): %s; worst hood under a beam %.1f deg -> %s%n",
        kind,
        file,
        alliance,
        report.isEmpty() ? "no trench stays" : String.join("; ", report),
        worst,
        violating ? "VIOLATION" : "ok");

    boolean enforce =
        kind.equals("bline") || "1".equals(System.getenv("AUDIT_SIM_ENFORCE_PATHPLANNER"));
    if (!enforce) {
      System.out.println(
          "[auto-audit] (PathPlanner sim is informational: results vary run to run)");
      return;
    }
    AuditBaseline baseline = AuditBaseline.load();
    if ("1".equals(System.getenv("AUDIT_UPDATE_BASELINE"))) {
      baseline.setSim(kind, file, violating);
      baseline.save();
      return;
    }
    boolean known = baseline.simKnown(kind, file);
    assertTrue(
        !violating || known,
        String.format(
            "NEW sim violation: %s '%s' had its hood up to %.1f deg under a trench beam (%s). Not"
                + " in baseline.json.",
            kind, file, worst, String.join("; ", report)));
    if (known && !violating) {
      System.out.println(
          "[auto-audit] FIXED in sim: " + kind + " '" + file + "' (remove from baseline)");
      if ("1".equals(System.getenv("AUDIT_STRICT_BASELINE"))) {
        assertTrue(false, "Fixed in sim but still in baseline.json: " + kind + " '" + file + "'");
      }
    }
  }

  private static NetworkTableEntry chooser(String key) {
    return NetworkTableInstance.getDefault()
        .getTable("SmartDashboard")
        .getSubTable("Rebuilt")
        .getSubTable("Auto Modes")
        .getEntry(key);
  }
}
