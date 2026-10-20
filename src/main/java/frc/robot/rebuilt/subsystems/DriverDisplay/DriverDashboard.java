package frc.robot.rebuilt.subsystems.DriverDisplay;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/**
 * Publishes the only values the drive team needs on the dashboard: the turret angle and the robot's
 * position on the field. Everything else is logged through AdvantageKit instead.
 *
 * <p>NetworkTables keys (used by {@code deploy/rebuilt_robot/elastic-layout.json}):
 *
 * <ul>
 *   <li>{@code /SmartDashboard/Driver/Turret Angle} – turret angle in degrees
 *   <li>{@code /SmartDashboard/Driver/Field} – Field2d showing only the robot pose
 * </ul>
 */
public class DriverDashboard extends SubsystemBase {
  private final Field2d field = new Field2d();
  private final Supplier<Pose2d> robotPose;
  private final Supplier<Angle> turretAngle;

  /**
   * Creates the driver dashboard.
   *
   * @param robotPose supplies the robot's current field pose
   * @param turretAngle supplies the turret's current angle
   */
  public DriverDashboard(Supplier<Pose2d> robotPose, Supplier<Angle> turretAngle) {
    this.robotPose = robotPose;
    this.turretAngle = turretAngle;
    SmartDashboard.putData("Driver/Field", field);
  }

  @Override
  public void periodic() {
    field.setRobotPose(robotPose.get());
    SmartDashboard.putNumber("Driver/Turret Angle", turretAngle.get().in(Degrees));
  }
}
