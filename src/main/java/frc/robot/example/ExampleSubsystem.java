package frc.robot.example;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.constants.GenericPID;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.frc5010.common.motors.MotorConstants.Motor;
import org.frc5010.common.motors.MotorFactory;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.function.AngularControlMotor;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.frc5010.common.motors.function.VelocityControlMotor;
import org.frc5010.common.motors.function.VerticalPositionControlMotor;
import org.frc5010.common.sensors.absolute_encoder.RevAbsoluteEncoder;
import org.frc5010.lobbinloco.FRC5010BallOnTheFly;
import org.littletonrobotics.junction.Logger;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation.IntakeSide;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;
import swervelib.simulation.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import swervelib.simulation.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeAlgaeOnFly;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;

public class ExampleSubsystem extends GenericSubsystem {
  protected PercentControlMotor motor;
  protected VelocityControlMotor controlledMotor;
  protected AngularControlMotor angularMotor;
  protected VerticalPositionControlMotor verticalMotor;
  protected IntakeSimulation intakeSimulation;
  protected NoteOnFly noteOnFly;
  protected RebuiltFuelOnFly fuelOnFly;
  protected GamePieceProjectile gamePieceProjectile;
  protected int scoredNotes = 0;
  protected Rotation2d rotation = new Rotation2d(Degrees.of(180));
  protected FlyWheel shooter;
  protected Arm arm;

  public ExampleSubsystem() {
    super("example.json");
    this.motor = (PercentControlMotor) devices.get("percent_motor");
    this.controlledMotor = (VelocityControlMotor) devices.get("velocity_motor");
    this.shooter = (FlyWheel) devices.get("Shooter");
    this.arm = (Arm) devices.get("Arm");

    this.angularMotor = angularControlledMotor();
    // verticalMotor = verticalControlledMotor();
    intakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "FRC5010Ball",
            GenericDrivetrain.getMapleSimDrive().get(),
            Inches.of(24.25),
            IntakeSide.FRONT,
            1);
  }

  public AngularControlMotor angularControlledMotor() {
    AngularControlMotor angularMotor =
        new AngularControlMotor(
                MotorFactory.Spark(13, Motor.Neo), "angular", getDisplayValuesHelper())
            .setupSimulatedMotor(
                (5.0 * 68.0 / 24.0) * (80.0 / 24.0),
                Units.lbsToKilograms(22),
                Inches.of(19),
                Degrees.of(0),
                Degrees.of(360),
                false,
                0,
                Degrees.of(0),
                false,
                0.1)
            .setVisualizer(mechanismSimulation, new Pose3d(0.75, 0, 0.25, new Rotation3d()));
    angularMotor.setEncoder(new RevAbsoluteEncoder((SparkMax) angularMotor.getMotor(), 360));
    angularMotor.setValues(new GenericPID(0.01, 0.000025, 0.003));
    angularMotor.setMotorFeedFwd(new MotorFeedFwdConstants(0.0, 0.01, 0.0, false));
    angularMotor.setIZone(3);
    angularMotor.setOutputRange(-12, 12);
    return angularMotor;
  }

  public Command getDefaultCommand(DoubleSupplier speed) {
    return Commands.run(() -> motor.set(speed.getAsDouble()), this);
  }

  public AngularVelocity getVelocity() {
    return shooter.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed) {
    return shooter.setSpeed(speed).alongWith(arm.setAngle(Degrees.of(30)).asProxy());
  }

  public Command setDutyCycle(double dutyCycle) {
    return shooter.set(dutyCycle).alongWith(arm.setAngle(Degrees.of(0)).asProxy());
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return shooter.setSpeed(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return shooter.set(dutyCycle);
  }

  public Command setPercentControlMotorReference(DoubleSupplier reference) {
    return Commands.runOnce(
        () -> {
          double speed = reference.getAsDouble();
          if (speed > 0.0 && !noteIsInsideIntake().getAsBoolean()) {
            intakeSimulation.startIntake();
          } else {
            intakeSimulation.stopIntake();
          }
          motor.set(speed);
        },
        this);
  }

  public Command sysIdShooter() {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(shooter.getMotor(), logPrefix, this), 5, 3, 3);
  }

  public Command setVelocityControlMotorReference(DoubleSupplier reference) {
    return Commands.runOnce(
        () -> {
          double speed = reference.getAsDouble();
          if (speed <= 0.0 && !noteIsInsideIntake().getAsBoolean()) {
            controlledMotor.setReference(speed);
          } else if (speed > 3000
              && noteIsInsideIntake().getAsBoolean()
              && obtainedGamePieceToScore().getAsBoolean()) {
            controlledMotor.setReference(speed);
            if (RobotBase.isSimulation()) {
              Pose2d worldPose = YAGSLSwerveDrivetrain.getSwerveDrive().getPose();
              gamePieceProjectile =
                  new ReefscapeAlgaeOnFly(
                      worldPose.getTranslation(),
                      controlledMotor.getRobotToMotor().getTranslation().toTranslation2d(),
                      YAGSLSwerveDrivetrain.getSwerveDrive().getFieldVelocity(),
                      worldPose.getRotation(),
                      Meters.of(0.45),
                      MetersPerSecond.of(speed / 6000 * 20),
                      Degrees.of(55));
              SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
            }
          } else if (speed < 3000
              && speed > 1000
              && noteIsInsideIntake().getAsBoolean()
              && obtainedGamePieceToScore().getAsBoolean()) {
            controlledMotor.setReference(speed);
            if (RobotBase.isSimulation()) {
              Pose2d worldPose = YAGSLSwerveDrivetrain.getSwerveDrive().getPose();
              gamePieceProjectile =
                  new ReefscapeAlgaeOnFly(
                      worldPose.getTranslation(),
                      controlledMotor.getRobotToMotor().getTranslation().toTranslation2d(),
                      YAGSLSwerveDrivetrain.getSwerveDrive().getFieldVelocity(),
                      worldPose.getRotation(),
                      Meters.of(0.45),
                      MetersPerSecond.of(speed / 6000 * 20),
                      Degrees.of(55));
              SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
            }
          } else {
            controlledMotor.setReference(speed);
          }
        },
        this);
  }

  public Command setAngularMotorReference(DoubleSupplier reference) {
    return Commands.runOnce(() -> angularMotor.setReference(reference.getAsDouble()), this);
  }

  public Trigger obtainedGamePieceToScore() {
    return new Trigger(
        () -> {
          return RobotBase.isSimulation()
              ? intakeSimulation.getGamePiecesAmount() == 1
                  && intakeSimulation.obtainGamePieceFromIntake()
              : false;
        });
  }

  public Trigger noteIsInsideIntake() {
    return new Trigger(
        () -> {
          return RobotBase.isSimulation() ? intakeSimulation.getGamePiecesAmount() > 0 : false;
        });
  }

  public Command addBallToRobot() {
    return Commands.runOnce(() -> intakeSimulation.addGamePieceToIntake());
  }

  public Command launchBall() {
    return Commands.runOnce(
        () -> {
          if (RobotBase.isSimulation()) {
            Pose2d worldPose = YAGSLSwerveDrivetrain.getSwerveDrive().getPose();
            gamePieceProjectile =
                new FRC5010BallOnTheFly(
                        worldPose.getTranslation(),
                        controlledMotor.getRobotToMotor().getTranslation().toTranslation2d(),
                        YAGSLSwerveDrivetrain.getSwerveDrive().getFieldVelocity(),
                        worldPose.getRotation(),
                        Meters.of(0.45),
                        MetersPerSecond.of(10),
                        Degrees.of(55))
                    .withProjectileTrajectoryDisplayCallBack(
                        (pose3ds) -> {
                          Logger.recordOutput(
                              logPrefix + "/GPTrajectory", pose3ds.toArray(Pose3d[]::new));
                        });
            SimulatedArena.getInstance().addGamePieceProjectile(gamePieceProjectile);
          }
        });
  }

  @Override
  public void periodic() {
    super.periodic();
    angularMotor.periodicUpdate();
    // verticalMotor.draw();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
    angularMotor.simulationUpdate();
    // verticalMotor.simulationUpdate();
  }
}
