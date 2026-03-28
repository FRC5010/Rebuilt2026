package frc.robot.rebuilt.subsystems.Indexer;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.rebuilt.commands.IndexerCommands;
import org.frc5010.common.arch.GenericSubsystem;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public AngularVelocity spindexerSpeed = Units.RadiansPerSecond.of(0.0);
    public double transferFrontSpeed = 0;
    public double transferBackSpeed = 0;
    public IndexerCommands.IndexerState stateRequested = IndexerCommands.IndexerState.IDLE;
    public IndexerCommands.IndexerState stateCurrent = IndexerCommands.IndexerState.IDLE;
  }

  public void runSpindexer(AngularVelocity speed);

  public void runTransferFront(double speed);

  // public void runTransferBack(double speed);

  public default void updateInputs(IndexerIOInputs inputs) {}

  public Command getSpindexerSysIdCommand(GenericSubsystem indexer);

  public Command getTransferSysIdCommand(GenericSubsystem indexer);
}
