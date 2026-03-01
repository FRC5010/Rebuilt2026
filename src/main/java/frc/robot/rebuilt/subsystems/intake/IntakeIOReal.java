package frc.robot.rebuilt.subsystems.Intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.littletonrobotics.junction.Logger;
import yams.mechanisms.positional.Arm;

public class IntakeIOReal implements IntakeIO {
  protected Map<String, Object> devices;
  private PercentControlMotor intakeLead;
  private PercentControlMotor intakeFollow;
  private Arm intakeHopper;

  /** initializes the intake and hopper */
  public IntakeIOReal(Map<String, Object> devices) {
    this.devices = devices;
    intakeLead = (PercentControlMotor) devices.get("intakeLead");
    intakeFollow = (PercentControlMotor) devices.get("intakeFollow");
    intakeLead.invert(true);
    intakeFollow.setFollow(intakeLead, false);
    intakeHopper = (Arm) devices.get("hopper");
  }

  @Override
  public void runIntake(double speed) {
    intakeLead.set(speed);
  }

  public void setHopperAngle(Angle angle) {
    intakeHopper.getMotorController().setPosition(angle);
  }

  public void setHopperPosition(Angle angle) {
    intakeHopper.getMotor().setEncoderPosition(angle);
  }

  public boolean isHopperMoving() {

    return Math.abs(
            intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second)))
        > 1.0;
  }

  public boolean isRetracted() {
    return (intakeHopper.getAngle().isNear(Degrees.of(120), Degrees.of(10)));
  }

  public boolean isDeployed() {
    return (intakeHopper.getAngle().isNear(Degrees.of(0.0), Degrees.of(10)));
  }

  public Command getHopperSysIdCommand() {
    return intakeHopper.sysId(Volts.of(4), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }
  /** Returns a sysid command for the hopper */
  public Command getHopperSysIdCommand(GenericSubsystem intake) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(
            intakeHopper.getMotorController(), intakeHopper.getName(), intake),
        5,
        5,
        3,
        () ->
            intakeHopper
                .isNear(
                    intakeHopper.getMotorController().getConfig().getMechanismUpperLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () ->
            intakeHopper
                .isNear(
                    intakeHopper.getMotorController().getConfig().getMechanismLowerLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () -> intakeHopper.getMotor().setDutyCycle(0));
  }

  public Command getHopperCharacterizationCommand(GenericSubsystem intake) {
    return SystemIdentification.feedforwardCharacterization(
        intake,
        (Voltage voltage) -> intakeHopper.getMotor().setVoltage(voltage),
        () -> intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
  }

  public void runHopper(double speed) {
    intakeHopper.getMotorController().setDutyCycle(speed);
  }
  /** updates the input structure with the current hopper and intake speed */
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    Logger.recordOutput(
        "Hopper Velocity",
        intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
    Logger.recordOutput("Hopper Moving", isHopperMoving());
    inputs.hopperAngle = intakeHopper.getMotorController().getMechanismPosition();
    inputs.hopperAngleDouble = inputs.hopperAngle.in(Degrees);
    inputs.speed = intakeLead.get();
    inputs.hopperAmps = intakeHopper.getMotor().getStatorCurrent().in(Amps);
  }
}
