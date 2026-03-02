// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt.subsystems.Launcher;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.FieldConstants;
import frc.robot.rebuilt.commands.LauncherCommands;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.subsystems.LEDStrip;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Pivot;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.simulation.Sensor;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/** Add your docs here. */
public class LauncherIOReal implements LauncherIO {

  protected Map<String, Object> devices;
  protected Pivot turret;
  protected Arm hood;
  protected GenericDrivetrain drivetrain;
  protected FlyWheel flyWheel;
  protected CANcoder crtEncoder40;
  protected CANcoder crtEncoder36;
  protected final Sensor crtSensor40;
  protected final Sensor crtSensor36;
  protected EasyCRT easyCrtSolver;
  /** Initializes the launcher hardware, encoders, simulated sensors, and angle solver */
  EasyCRTConfig easyCrt;

  public LauncherIOReal(Map<String, Object> devices, Map<String, GenericSubsystem> subsystems) {
    this.devices = devices;
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    turret = (Pivot) devices.get("turret");
    hood = (Arm) devices.get("hood");
    flyWheel = (FlyWheel) devices.get("flywheel");
    CANBus canivoreBus = new CANBus("canivore");
    crtEncoder40 = new CANcoder(21, canivoreBus);
    crtEncoder36 = new CANcoder(22, canivoreBus);
    double sensor40Sim = 0.391;
    double sensor36Sim = 0.274;
    crtSensor40 =
        new SensorConfig("CRT sensor 40")
            .withField("angle", () -> crtEncoder40.getAbsolutePosition().getValueAsDouble(), 0.0)
            .withSimulatedValue("angle", Seconds.of(0), Seconds.of(0.5), sensor40Sim)
            .getSensor();
    crtSensor36 =
        new SensorConfig("CRT sensor 36")
            .withField("angle", () -> crtEncoder36.getAbsolutePosition().getValueAsDouble(), 0.0)
            .withSimulatedValue("angle", Seconds.of(0), Seconds.of(0.5), sensor36Sim)
            .getSensor();

    easyCrt =
        new EasyCRTConfig(
                () -> Rotations.of(crtSensor40.getAsDouble("angle")),
                () -> Rotations.of(crtSensor36.getAsDouble("angle")))
            .withCommonDriveGear(
                /* commonRatio (mech:drive) */ 30.0,
                /* driveGearTeeth */ 12,
                /* encoder1Pinion */ 40,
                /* encoder2Pinion */ 36)
            .withAbsoluteEncoderOffsets(
                Rotations.of(-0.244), Rotations.of(-0.210)) // set after mechanical zero
            .withMechanismRange(Degrees.of(-168), Degrees.of(173)) // -360 deg to +720 deg
            .withMatchTolerance(Rotations.of(0.06)) // ~1.08 deg at encoder2 for the example ratio
            .withAbsoluteEncoderInversions(true, false)
            .withCrtGearRecommendationConstraints(
                /* coverageMargin */ 1.2,
                /* minTeeth */ 15,
                /* maxTeeth */ 45,
                /* maxIterations */ 30);

    easyCrtSolver = new EasyCRT(easyCrt);
    // // Test Values
    SmartDashboard.putNumber(
        "Unique Coverage", easyCrt.getUniqueCoverage().orElse(Degrees.of(0.0)).in(Degrees));
    SmartDashboard.putBoolean("Coverage Satisfies Range", easyCrt.coverageSatisfiesRange());
    SmartDashboard.putNumber("EasyCRT Enc 1", easyCrt.getAbsoluteEncoder1Angle().in(Degrees));
    SmartDashboard.putNumber(
        "EasyCRT Enc 1 Ratio", easyCrt.getEncoder1RotationsPerMechanismRotation());
    SmartDashboard.putNumber("EasyCRT Enc 2", easyCrt.getAbsoluteEncoder2Angle().in(Degrees));
    SmartDashboard.putNumber(
        "EasyCRT Enc 2 Ratio", easyCrt.getEncoder2RotationsPerMechanismRotation());
    Angle calculatedAngle = easyCrtSolver.getAngleOptional().orElse(Degrees.of(0.0));
    SmartDashboard.putNumber("CRT Angle", calculatedAngle.in(Degrees));
    SmartDashboard.putString("CRT Status", easyCrtSolver.getLastStatus().name());
    SmartDashboard.putNumber("CRT Error Rot", easyCrtSolver.getLastErrorRotations());
    turret.getMotor().setEncoderPosition(calculatedAngle);
  }

  @Override()
  /** Updating launcher sensor data, calculates shot parameters, and populates input telemetry */
  public void updateInputs(LauncherIOInputs inputs) {
    SmartDashboard.putNumber("Encoder 40", crtSensor40.getAsDouble("angle"));
    SmartDashboard.putNumber("EasyCRT Enc 2", easyCrt.getAbsoluteEncoder2Angle().in(Degrees));
    SmartDashboard.putNumber("Encoder 36", crtSensor36.getAsDouble("angle"));
    SmartDashboard.putNumber("EasyCRT Enc 1", easyCrt.getAbsoluteEncoder1Angle().in(Degrees));
    // Angle calculatedAngle = easyCrtSolver.getAngleOptional().orElse(Degrees.of(0.0));
    // SmartDashboard.putNumber("CRT Angle", calculatedAngle.in(Degrees));
    // SmartDashboard.putString("CRT Status", easyCrtSolver.getLastStatus().name());
    // SmartDashboard.putNumber("CRT Error Rot", easyCrtSolver.getLastErrorRotations());

    ShotCalculator.getInstance().clearShootingParameters();
    ShotCalculator.ShootingParameters params =
        ShotCalculator.getInstance()
            .getParameters(
                turret
                    .getPivotConfig()
                    .getMechanismPositionConfig()
                    .getRelativePosition()
                    .get()
                    .toTranslation2d(),
                Rotation2d.fromDegrees(turret.getAngle().in(Degrees)));
    inputs.isValidCalculation = false;
    if (params != null) {
      inputs.isValidCalculation = params.isValid();
      inputs.hoodAngleCalculated = Radian.of(params.hoodAngle());
      inputs.turretAngleCalculated = params.turretAngle().getMeasure();
      inputs.flyWheelSpeedCalculated = RPM.of(params.flywheelSpeed() * 0.45);
      inputs.distanceToVirtualTarget = params.distanceToVirtualTarget();
    }
    /** Reads the desired flywheel, hood, and turret setpoints */
    inputs.flyWheelSpeedDesired =
        flyWheel
            .getMotorController()
            .getMechanismSetpointVelocity()
            .map(it -> it)
            .orElse(RPM.of(0.0));
    inputs.hoodAngleDesired =
        hood.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));
    inputs.turretAngleDesired =
        turret.getMotorController().getMechanismPositionSetpoint().orElse(Degrees.of(0.0));

    inputs.flyWheelSpeedActual = flyWheel.getSpeed();
    inputs.hoodAngleActual = hood.getAngle();
    inputs.turretAngleActual = turret.getAngle();

    inputs.flyWheelSpeedError = inputs.flyWheelSpeedActual.minus(inputs.flyWheelSpeedDesired);
    inputs.hoodAngleError = inputs.hoodAngleActual.minus(inputs.hoodAngleDesired).in(Degrees);
    inputs.turretAngleError = inputs.turretAngleActual.minus(inputs.turretAngleDesired).in(Degrees);

    inputs.flyWheelSpeedAtGoal =
        Math.abs(inputs.flyWheelSpeedError.in(RPM)) <= Constants.Launcher.SHOOTER_TOLERANCE_RPM;
    inputs.hoodAngleAtGoal =
        Math.abs(inputs.hoodAngleError) <= Constants.Launcher.HOOD_ANGLE_TOLERANCE_DEGREES;
    inputs.turretAngleAtGoal =
        Math.abs(inputs.turretAngleError) <= Constants.Launcher.TURRET_ANGLE_TOLERANCE_DEGREES;

    inputs.hoodVelocity = hood.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.turretVelocity =
        turret.getMotorController().getMechanismVelocity().in(Degrees.per(Second));
    inputs.flyWheelMotorOutput = flyWheel.getMotor().getStatorCurrent().in(Amps);

    inputs.robotToTarget = LauncherCommands.getRobotToTarget();

    inputs.targetDistance = Meters.of(inputs.robotToTarget.getDistance(new Translation2d()));

    // Log trench detection every cycle
    isNearTrench();
  }
  /** Configuring the shot calculator with limits and constraints */
  @Override
  public void configureShotCalculator(ShotCalculator shotCalculator) {
    shotCalculator.setShotTables(ShotCalculator.createDefaultTables());
    shotCalculator.setTurretConstraints(
        Rotation2d.fromDegrees(
            turret.getMotorController().getConfig().getMechanismLowerLimit().get().in(Degrees)),
        Rotation2d.fromDegrees(
            turret.getMotorController().getConfig().getMechanismUpperLimit().get().in(Degrees)),
        Rotation2d.fromDegrees(10.0));
  }
  /** Sets the flywheel motor's duty cycle */
  public void runShooter(double speed) {
    flyWheel.getMotor().setDutyCycle(speed);
  }
  /** Sets the flywheel motor's angular velocity */
  public void setFlyWheelVelocity(AngularVelocity speed) {
    flyWheel.getMotor().setVelocity(speed);
  }
  /** Sets the hood angle and overrides the requested angle if the hood is near the trench */
  public void setHoodAngle(Angle angle) {
    if (isNearTrench()) {
      hood.getMotorController().setPosition(Degrees.of(31.0));
    } else {
      hood.getMotorController().setPosition(angle);
    }
  }
  /** Sets the low hard limit to 30 degrees and updates LED's */
  public void setHoodAngleLow() {
    hood.getMotorController()
        .setPosition(hood.getArmConfig().getLowerHardLimit().orElse(Degrees.of(30)));
    LEDStrip.changeSegmentPattern(ConfigConstants.ALL_LEDS, LEDStrip.getSolidPattern(Color.kGreen));
  }
  /** Sets the angle of the turret based on the motor request */
  public void setTurretRotation(Angle angle) {
    turret.getMotorController().setPosition(angle);
  }
  /** Converts the flywheel angular velocity into speed */
  public LinearVelocity getFlyWheelExitSpeed(AngularVelocity velocity) {
    return MetersPerSecond.of(
        flyWheel.getShooterConfig().getCircumference().in(Meters)
            * (velocity.in(RotationsPerSecond)));
  }
  /** Returns SysId command for the hood */
  public Command getHoodSysIdCommand() {
    return hood.sysId(Volts.of(4), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }

  /** Runs sysid for the cahracterized hood motor and stops at limits */
  public Command getHoodSysIdCommand(GenericSubsystem launcher) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(hood.getMotorController(), hood.getName(), launcher),
        5,
        3,
        3,
        () ->
            hood.isNear(
                    hood.getMotorController().getConfig().getMechanismUpperLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () ->
            hood.isNear(
                    hood.getMotorController().getConfig().getMechanismLowerLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () -> hood.getMotor().setDutyCycle(0));
  }

  public Command getTurretSysIdCommand() {
    return turret.sysId(Volts.of(4), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }
  /** Characterizes the turret */
  public Command getTurretSysIdCommand(GenericSubsystem launcher) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(
            turret.getMotorController(), turret.getName(), launcher),
        5,
        3.5,
        3,
        () ->
            turret
                .isNear(
                    turret.getMotorController().getConfig().getMechanismUpperLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () ->
            turret
                .isNear(
                    turret.getMotorController().getConfig().getMechanismLowerLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () -> turret.getMotor().setDutyCycle(0));
  }
  /** Applies voltage and measures hood velocity to characterize the feed forward */
  public Command getHoodCharacterizationCommand(GenericSubsystem launcher) {
    return SystemIdentification.feedforwardCharacterization(
        launcher,
        (Voltage voltage) -> hood.getMotor().setVoltage(voltage),
        () -> hood.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
  }
  /** Applies voltage and measures turret velocity to characterize the feedfoward */
  public Command getTurretCharacterizationCommand(GenericSubsystem launcher) {
    return SystemIdentification.feedforwardCharacterization(
        launcher,
        (Voltage voltage) -> turret.getMotor().setVoltage(voltage),
        () -> turret.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
  }
  /** sets the flywheel, hood, and turret motor duty cycles to 0, which stops the motors */
  public void stopAllMotors() {
    flyWheel.getMotor().setDutyCycle(0);
    hood.getMotor().setDutyCycle(0);
    turret.getMotor().setDutyCycle(0);
  }

  public Command getFlyWheelSysIdCommand() {
    return flyWheel.sysId(Volts.of(8), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }

  private boolean isNearTrench() {
    Pose2d current = drivetrain.getPoseEstimator().getCurrentPose();
    double currentX = current.getX();
    double currentY = current.getY();

    double topTrenchLeftX = FieldConstants.TrenchZoneTop.nearAllianceLeftDanger.getX();
    double topTrenchRightX = FieldConstants.TrenchZoneTop.nearAllianceRightDanger.getX();

    double topTrenchY = FieldConstants.TrenchZoneTop.nearAllianceLeftDanger.getY();

    double topOppTrenchLeftX = FieldConstants.TrenchZoneTop.oppAllianceLeftDanger.getX();
    double topOppTrenchRightX = FieldConstants.TrenchZoneTop.oppAllianceRightDanger.getX();

    double lowerTrenchLeftX = FieldConstants.TrenchZoneBottom.nearAllianceLeftDanger.getX();
    double lowerTrenchRightX = FieldConstants.TrenchZoneBottom.nearAllianceRightDanger.getX();

    double lowerTrenchY = FieldConstants.TrenchZoneBottom.oppAllianceLeftDanger.getY();

    double lowerOppTrenchLeftX = FieldConstants.TrenchZoneBottom.oppAllianceLeftDanger.getX();
    double lowerOppTrenchRightX = FieldConstants.TrenchZoneBottom.oppAllianceRightDanger.getX();

    boolean nearAllianceTop =
        ((currentX > topTrenchLeftX && currentX < topTrenchRightX) && currentY > topTrenchY);

    boolean nearOppAllianceTop =
        ((currentX > topOppTrenchLeftX && currentX < topOppTrenchRightX) && currentY > topTrenchY);

    boolean nearAllianceBottom =
        ((currentX > lowerTrenchLeftX && currentX < lowerTrenchRightX) && currentY < lowerTrenchY);

    boolean nearOppAllianceBottom =
        ((currentX > lowerOppTrenchLeftX && currentX < lowerOppTrenchRightX)
            && currentY < lowerTrenchY);

    SmartDashboard.putBoolean("Near Top Opp Alliance", nearOppAllianceTop);
    SmartDashboard.putBoolean("Near Top Alliance", nearAllianceTop);
    SmartDashboard.putBoolean("Near Bottom Opp Alliance", nearOppAllianceBottom);
    SmartDashboard.putBoolean("Near Bottom Alliance", nearAllianceBottom);

    if (nearAllianceTop || nearOppAllianceTop || nearAllianceBottom || nearOppAllianceBottom)
      return true;

    return false;
  }

  public Command getFlyWheelSysIdCommand(GenericSubsystem launcher) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(
            flyWheel.getMotorController(), flyWheel.getName(), launcher),
        8,
        3,
        3);
  }
}
