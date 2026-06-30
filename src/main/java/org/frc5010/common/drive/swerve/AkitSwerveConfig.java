// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive.swerve;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.devices.DrivetrainConstantsJson;
import org.frc5010.common.config.json.devices.MotorSystemIdJson;
import org.frc5010.common.drive.SwerveDriveConfig;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/** Add your docs here. */
public class AkitSwerveConfig extends SwerveDriveConfig {
  public final SwerveDrivetrainConstants DrivetrainConstants;

  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;
  public double ODOMETRY_FREQUENCY;
  public final double DRIVE_BASE_RADIUS;
  protected SwerveModuleConstants[] MODULES;
  private final CANBus kCANBus;

  public AkitSwerveConfig(AkitTalonFXSwerveConfigBuilder builder) {
    super(builder);
    this.DrivetrainConstants = builder.DrivetrainConstants;
    this.FrontLeft = builder.FrontLeft;
    this.FrontRight = builder.FrontRight;
    this.BackLeft = builder.BackLeft;
    this.BackRight = builder.BackRight;
    MODULES = new SwerveModuleConstants[] {FrontLeft, FrontRight, BackLeft, BackRight};
    this.kCANBus = builder.kCANBus;
    DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(FrontLeft.LocationX, FrontLeft.LocationY),
                Math.hypot(FrontRight.LocationX, FrontRight.LocationY)),
            Math.max(
                Math.hypot(BackLeft.LocationX, BackLeft.LocationY),
                Math.hypot(BackRight.LocationX, BackRight.LocationY)));
  }

  public SwerveModuleConstants getModuleConstants(int index) {
    return MODULES[index];
  }

  public static AkitSwerveConfig builder(
      DrivetrainConstantsJson constants, SubsystemBase subsystem) {
    return new AkitSwerveConfig(new AkitTalonFXSwerveConfigBuilder(constants, subsystem));
  }

  public CANBus getCANBus() {
    return kCANBus;
  }

  public static class AkitTalonFXSwerveConfigBuilder
      extends SwerveDriveConfig.SwerveDriveConfigBuilder {
    public final SwerveDrivetrainConstants DrivetrainConstants;

    private final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        ConstantCreator;

    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontLeft;
    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        FrontRight;
    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackLeft;
    public final SwerveModuleConstants<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        BackRight;
    public final CANBus kCANBus;

    public AkitTalonFXSwerveConfigBuilder(
        DrivetrainConstantsJson constants, SubsystemBase subsystem) {
      super();
      kCANBus = new CANBus(constants.canbus, "./logs/example.hoot");
      bumperFrameLength = UnitsParser.parseDistance(constants.bumperFrameLength);
      bumperFrameWidth = UnitsParser.parseDistance(constants.bumperFrameWidth);
      maxDriveSpeed = UnitsParser.parseVelocity(constants.maxDriveSpeed);
      robotMass = UnitsParser.parseMass(constants.robotMass);
      driveInertia = UnitsParser.parseMomentOfInertia(constants.driveInertia);
      steerInertia = UnitsParser.parseMomentOfInertia(constants.steerInertia);
      canbus = Constants.CURRENT_MODE == Constants.SIM_MODE ? "" : constants.canbus;
      wheelDiameter = UnitsParser.parseDistance(constants.wheelDiameter);
      driveGearRatio =
          new MechanismGearing(GearBox.fromStages(constants.driveGearRatio))
              .getRotorToMechanismRatio();
      steerGearRatio =
          new MechanismGearing(GearBox.fromStages(constants.steerGearRatio))
              .getRotorToMechanismRatio();
      DrivetrainConstants =
          new SwerveDrivetrainConstants()
              .withCANBusName(kCANBus.getName())
              .withPigeon2Id(constants.gyro.id)
              .withPigeon2Configs(null);

      CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(
                  new MechanismGearing(GearBox.fromStages(constants.driveGearRatio))
                      .getRotorToMechanismRatio())
              .withSteerMotorGearRatio(
                  new MechanismGearing(GearBox.fromStages(constants.steerGearRatio))
                      .getRotorToMechanismRatio())
              .withCouplingGearRatio(constants.coupleRatio)
              .withWheelRadius(UnitsParser.parseDistance(constants.wheelDiameter).div(2))
              .withSteerMotorGains(steerSlot0Configs(constants.steerMotorControl))
              .withDriveMotorGains(driveSlot0Configs(constants.driveMotorControl))
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput(constants.steerMotorControl))
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput(constants.driveMotorControl))
              .withSlipCurrent(UnitsParser.parseAmps(constants.slipCurrent))
              .withSpeedAt12Volts(UnitsParser.parseVelocity(constants.maxDriveSpeed))
              .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
              .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withDriveMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs().withSupplyCurrentLimit(Amps.of(30))))
              .withSteerMotorInitialConfigs(
                  new TalonFXConfiguration()
                      .withCurrentLimits(
                          new CurrentLimitsConfigs()
                              // Swerve azimuth does not require much torque output, so we can set
                              // arelatively low stator current limit to help avoid brownouts
                              // without impacting performance.
                              .withStatorCurrentLimit(Amps.of(30))
                              .withStatorCurrentLimitEnable(true)))
              .withEncoderInitialConfigs(encoderInitialConfigs)
              .withSteerInertia(UnitsParser.parseMomentOfInertia(constants.steerInertia))
              .withDriveInertia(UnitsParser.parseMomentOfInertia(constants.driveInertia))
              .withSteerFrictionVoltage(constants.steerMotorControl.feedForward.s)
              .withDriveFrictionVoltage(constants.driveMotorControl.feedForward.s);
      trackWidth = UnitsParser.parseDistance(constants.trackWidth);
      wheelBase = UnitsParser.parseDistance(constants.wheelBase);
      Distance kFrontLeftXPos = trackWidth.div(2);
      Distance kFrontLeftYPos = wheelBase.div(2);
      FrontLeft =
          ConstantCreator.createModuleConstants(
              constants.modules.get("frontLeft").steerMotorSetup.canId,
              constants.modules.get("frontLeft").driveMotorSetup.canId,
              constants.modules.get("frontLeft").encoderId,
              UnitsParser.parseAngle(constants.modules.get("frontLeft").absoluteOffset),
              kFrontLeftXPos,
              kFrontLeftYPos,
              constants.invertLeftSide,
              constants.modules.get("frontLeft").steerMotorSetup.inverted,
              constants.modules.get("frontLeft").encoderInverted);
      Distance kFrontRightXPos = trackWidth.div(2);
      Distance kFrontRightYPos = wheelBase.div(-2);
      FrontRight =
          ConstantCreator.createModuleConstants(
              constants.modules.get("frontRight").steerMotorSetup.canId,
              constants.modules.get("frontRight").driveMotorSetup.canId,
              constants.modules.get("frontRight").encoderId,
              UnitsParser.parseAngle(constants.modules.get("frontRight").absoluteOffset),
              kFrontRightXPos,
              kFrontRightYPos,
              constants.invertRightSide,
              constants.modules.get("frontRight").steerMotorSetup.inverted,
              constants.modules.get("frontRight").encoderInverted);

      Distance kBackLeftXPos = trackWidth.div(-2);
      Distance kBackLeftYPos = wheelBase.div(2);
      BackLeft =
          ConstantCreator.createModuleConstants(
              constants.modules.get("backLeft").steerMotorSetup.canId,
              constants.modules.get("backLeft").driveMotorSetup.canId,
              constants.modules.get("backLeft").encoderId,
              UnitsParser.parseAngle(constants.modules.get("backLeft").absoluteOffset),
              kBackLeftXPos,
              kBackLeftYPos,
              constants.invertLeftSide,
              constants.modules.get("backLeft").steerMotorSetup.inverted,
              constants.modules.get("backLeft").encoderInverted);

      Distance kBackRightXPos = trackWidth.div(-2);
      Distance kBackRightYPos = wheelBase.div(-2);
      BackRight =
          ConstantCreator.createModuleConstants(
              constants.modules.get("backRight").steerMotorSetup.canId,
              constants.modules.get("backRight").driveMotorSetup.canId,
              constants.modules.get("backRight").encoderId,
              UnitsParser.parseAngle(constants.modules.get("backRight").absoluteOffset),
              kBackRightXPos,
              kBackRightYPos,
              constants.invertRightSide,
              constants.modules.get("backRight").steerMotorSetup.inverted,
              constants.modules.get("backRight").encoderInverted);
    }

    // --- Sim defaults (applied when running in sim and JSON does not override). --------------
    //
    // Real hardware uses TorqueCurrentFOC with the gains parsed from JSON. Maple-sim's motor model
    // doesn't track those gains cleanly (amps-shaped JSON numbers either bang-bang or wildly
    // under-drive when interpreted as volts, and saturate the slip-current limit when kept as
    // amps), so sim defaults to Voltage closed-loop with these reference gains. Each motor's
    // optional {@code sim} sub-block in JSON can override any of feedBack / feedForward /
    // closedLoopOutput.
    private static final MotorSystemIdJson.FeedBack DEFAULT_STEER_SIM_FB = makeFB(70, 0, 4.5);
    private static final MotorSystemIdJson.FeedForward DEFAULT_STEER_SIM_FF = makeFF(0, 1.91, 0);
    private static final MotorSystemIdJson.FeedBack DEFAULT_DRIVE_SIM_FB = makeFB(0.05, 0, 0);
    // kV ≈ 12 V / 16.7 wheel rev/s (Kraken X60 free speed ÷ 6:1) — feedforward delivers ~85% of
    // max-speed authority on its own.
    private static final MotorSystemIdJson.FeedForward DEFAULT_DRIVE_SIM_FF = makeFF(0.1, 0.72, 0);
    private static final ClosedLoopOutputType DEFAULT_SIM_OUTPUT = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DEFAULT_REAL_OUTPUT =
        ClosedLoopOutputType.TorqueCurrentFOC;

    private static MotorSystemIdJson.FeedBack makeFB(double p, double i, double d) {
      MotorSystemIdJson.FeedBack fb = new MotorSystemIdJson.FeedBack();
      fb.p = p;
      fb.i = i;
      fb.d = d;
      return fb;
    }

    private static MotorSystemIdJson.FeedForward makeFF(double s, double v, double a) {
      MotorSystemIdJson.FeedForward ff = new MotorSystemIdJson.FeedForward();
      ff.s = s;
      ff.v = v;
      ff.a = a;
      return ff;
    }

    private static Slot0Configs steerSlot0Configs(MotorSystemIdJson motor) {
      return buildSlot0(motor, DEFAULT_STEER_SIM_FB, DEFAULT_STEER_SIM_FF)
          .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    }

    private static Slot0Configs driveSlot0Configs(MotorSystemIdJson motor) {
      return buildSlot0(motor, DEFAULT_DRIVE_SIM_FB, DEFAULT_DRIVE_SIM_FF);
    }

    private static Slot0Configs buildSlot0(
        MotorSystemIdJson motor,
        MotorSystemIdJson.FeedBack simDefaultFB,
        MotorSystemIdJson.FeedForward simDefaultFF) {
      MotorSystemIdJson.FeedBack fb = motor.feedBack;
      MotorSystemIdJson.FeedForward ff = motor.feedForward;
      if (RobotBase.isSimulation()) {
        MotorSystemIdJson.Sim sim = motor.sim;
        fb = (sim != null && sim.feedBack != null) ? sim.feedBack : simDefaultFB;
        ff = (sim != null && sim.feedForward != null) ? sim.feedForward : simDefaultFF;
      }
      return new Slot0Configs()
          .withKP(fb.p)
          .withKI(fb.i)
          .withKD(fb.d)
          .withKS(ff.s)
          .withKV(ff.v)
          .withKA(ff.a);
    }

    private static ClosedLoopOutputType steerClosedLoopOutput(MotorSystemIdJson motor) {
      return resolveClosedLoopOutput(motor);
    }

    private static ClosedLoopOutputType driveClosedLoopOutput(MotorSystemIdJson motor) {
      return resolveClosedLoopOutput(motor);
    }

    private static ClosedLoopOutputType resolveClosedLoopOutput(MotorSystemIdJson motor) {
      if (RobotBase.isSimulation()) {
        if (motor.sim != null && motor.sim.closedLoopOutput != null) {
          return parseClosedLoopOutput(motor.sim.closedLoopOutput, DEFAULT_SIM_OUTPUT);
        }
        return DEFAULT_SIM_OUTPUT;
      }
      return DEFAULT_REAL_OUTPUT;
    }

    private static ClosedLoopOutputType parseClosedLoopOutput(
        String spec, ClosedLoopOutputType fallback) {
      if ("voltage".equalsIgnoreCase(spec)) return ClosedLoopOutputType.Voltage;
      if ("torquecurrentfoc".equalsIgnoreCase(spec)
          || "torque_current".equalsIgnoreCase(spec)
          || "torquecurrent".equalsIgnoreCase(spec)) {
        return ClosedLoopOutputType.TorqueCurrentFOC;
      }
      return fallback;
    }
  }
}
