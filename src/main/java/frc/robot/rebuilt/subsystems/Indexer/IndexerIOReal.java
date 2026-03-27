package frc.robot.rebuilt.subsystems.Indexer;

import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Map;
import yams.mechanisms.velocity.FlyWheel;

/** Implements the hardware Indexer IO */
public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;
  private FlyWheel spindexer;
  // private PercentControlMotor transferFront, transferBack;
  private FlyWheel transferFront;

  public IndexerIOReal(Map<String, Object> devices) {
    spindexer = (FlyWheel) devices.get("spindexer");
    transferFront = (FlyWheel) devices.get("transfer");
    // transferFront = (PercentControlMotor) devices.get("transfer_front");
    // transferBack = (PercentControlMotor) devices.get("transfer_back");
    // transferFront.invert(true);
    // transferFront.setFollow(transferBack, false);
    this.devices = devices;
  }
  /** Updates indexer input values with current motor speed */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerSpeed = spindexer.getMotor().getMechanismVelocity();
    inputs.transferFrontSpeed = transferFront.getMotor().getDutyCycle();
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
    transferFront.getMotor().setDutyCycle(speed);
  }

  // @Override
  // public void runTransferBack(double speed) {
  //   transferBack.set(speed);
  // }
}
