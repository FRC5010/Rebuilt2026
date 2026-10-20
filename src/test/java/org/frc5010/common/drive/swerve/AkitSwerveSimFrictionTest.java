package org.frc5010.common.drive.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import org.frc5010.common.config.json.AKitSwerveDrivetrainJson;
import org.frc5010.common.drive.swerve.akit.util.PhoenixUtil;
import org.junit.jupiter.api.Test;

/**
 * Guards the simulation friction voltages.
 *
 * <p>The deploy JSON's drive/steer {@code feedForward.s} are TorqueCurrentFOC <em>amps</em> (about
 * 4 A). They were once passed to the physics sim as friction <em>volts</em>, which put a ~4 V
 * dead-band on the simulated drive: it undershot low commanded speeds (0.5 m/s commanded gave ~0.2
 * m/s) and coasted ~0.2 m past path end points, while the real robot tracked well.
 */
public class AkitSwerveSimFrictionTest {
  private static AKitSwerveDrivetrainJson load() throws Exception {
    return new ObjectMapper()
        .readValue(
            new File("src/main/deploy/rebuilt_robot/akit_swerve_drivetrain.json"),
            AKitSwerveDrivetrainJson.class);
  }

  @Test
  public void simFrictionVoltagesAreSimParametersNotRealKs() throws Exception {
    AKitSwerveDrivetrainJson json = load();
    AkitSwerveConfig config = AkitSwerveConfig.builder(json.constants, null);

    assertEquals(
        PhoenixUtil.SIM_DRIVE_FRICTION_VOLTAGE.baseUnitMagnitude(),
        config.FrontLeft.DriveFrictionVoltage,
        1e-9,
        "drive friction voltage must come from the sim constant, not feedForward.s (amps)");
    assertEquals(
        PhoenixUtil.SIM_STEER_FRICTION_VOLTAGE.baseUnitMagnitude(),
        config.FrontLeft.SteerFrictionVoltage,
        1e-9,
        "steer friction voltage must come from the sim constant, not feedForward.s (amps)");
  }

  @Test
  public void simFrictionVoltagesAreSmall() throws Exception {
    AkitSwerveConfig config = AkitSwerveConfig.builder(load().constants, null);
    for (double v :
        new double[] {
          config.FrontLeft.DriveFrictionVoltage, config.FrontLeft.SteerFrictionVoltage
        }) {
      assertTrue(
          v > 0.0 && v < 1.0, "sim friction voltage should be a fraction of a volt, got " + v);
    }
  }

  @Test
  public void realRobotKsIsStillCurrentScale() throws Exception {
    // Documents why the two must not be mixed: the characterised kS is several amps.
    assertTrue(
        load().constants.driveMotorControl.feedForward.s > 1.0,
        "if kS is now volt-scale this guard (and the sim constants) should be revisited");
  }
}
