package frc.robot.rebuilt.subsystems.Indexer;

import java.util.Map;
import org.frc5010.common.motors.function.PercentControlMotor;
import yams.mechanisms.velocity.FlyWheel;

/** Implements the hardware Indexer IO */
public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;
  private PercentControlMotor spindexer;
  private FlyWheel transferFront;

  public IndexerIOReal(Map<String, Object> devices) {
    spindexer = (PercentControlMotor) devices.get("spindexer");
    transferFront = (FlyWheel) devices.get("transfer");
    this.devices = devices;
  }

  /** Updates indexer input values with current motor speed */
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerSpeed = spindexer.get();
    inputs.transferFrontSpeed = transferFront.getMotor().getDutyCycle();
  }

  /** Sets the spindexer motor speed */
  @Override
  public void runSpindexer(double speed) {
    spindexer.set(speed);
  }

  /** Sets the front transfer motor speed */
  @Override
  public void runTransferFront(double speed) {
    // transferFront.set(speed);
    transferFront.getMotor().setDutyCycle(speed);
  }
}
