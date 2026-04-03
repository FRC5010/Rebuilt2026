package frc.robot.rebuilt.subsystems.Indexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.rebuilt.commands.IndexerCommands;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double spindexerSpeed = 0;
    public double transferFrontSpeed = 0;
    public AngularVelocity transferFrontActual = RPM.of(0);
    public double transferBackSpeed = 0;
    public IndexerCommands.IndexerState stateRequested = IndexerCommands.IndexerState.IDLE;
    public IndexerCommands.IndexerState stateCurrent = IndexerCommands.IndexerState.IDLE;
  }

  public void runSpindexer(double speed);

  public void runTransferFront(double speed);

  // public void runTransferBack(double speed);

  public default void updateInputs(IndexerIOInputs inputs) {}
}
