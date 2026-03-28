package frc.robot.rebuilt.subsystems.Indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.SystemIdentification;
import yams.mechanisms.velocity.FlyWheel;

/** Implements the hardware Indexer IO */
public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;
  private FlyWheel spindexer;
  private FlyWheel transfer;

  public IndexerIOReal(Map<String, Object> devices) {
    spindexer = (FlyWheel) devices.get("spindexer");
    transfer = (FlyWheel) devices.get("transfer");
    this.devices = devices;
  }
  /** Updates indexer input values with current motor speed */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerSpeed = spindexer.getMotor().getMechanismVelocity();
    inputs.transferFrontSpeed = transfer.getMotor().getDutyCycle();
    // inputs.transferFrontSpeed = transferFront.get();
    // inputs.transferBackSpeed = transferBack.get();
  }
  /** Sets the spindexer motor speed */
  @Override
  public void runSpindexer(AngularVelocity speed) {
    // spindexer.set(speed);
    spindexer.getMotor().setVelocity(speed);
    ;
  }
  /** Sets the front transfer motor speed */
  @Override
  public void runTransferFront(double speed) {
    // transferFront.set(speed);
    transfer.getMotor().setDutyCycle(speed);
  }

  public Command getSpindexerSysIdCommand(GenericSubsystem indexer) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(
            spindexer.getMotorController(), spindexer.getName(), indexer),
        8,
        3,
        3);
  }

  public Command getTransferSysIdCommand(GenericSubsystem indexer) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.rpmSysIdRoutine(
            transfer.getMotorController(), transfer.getName(), indexer),
        8,
        3,
        3);
  }
}
