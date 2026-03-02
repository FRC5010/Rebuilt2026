package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.commands.IndexerCommands.IndexerState;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.subsystems.LEDStrip;

public class IndexerCommands {
  /** declares variables that will later hold state objects */
  private Map<String, GenericSubsystem> subsystems;

  private StateMachine stateMachine;
  private State idleState;
  private State churnState;
  private State feedState;
  private State forceState;
  private static Indexer indexer;

  /** defines possible states of the indexer */
  public static enum IndexerState {
    IDLE,
    CHURN,
    FORCE,
    FEED
  }

  /** Stores the subsystem map and retrieves the indexer instance */
  public IndexerCommands(Map<String, GenericSubsystem> systems) {
    this.subsystems = systems;
    IndexerCommands.indexer = (Indexer) subsystems.get(Constants.INDEXER);
    configureTriggerStates();
    // configureStateMachine();
  }

  /** Configures the state machine */
  private void configureStateMachine() {
    stateMachine = new StateMachine("IndexStateMachine");
    idleState = stateMachine.addState("idle", idleStateCommand());
    if (indexer != null) {
      /** Adds churn, force, and feed states if there is an indexer */
      churnState = stateMachine.addState("churn", churnStateCommand());
      feedState = stateMachine.addState("feed", feedStateCommand());
      forceState =
          stateMachine.addState(
              "force",
              Commands.runOnce(
                  () -> {
                    indexer.setCurrentState(IndexerState.FORCE);
                    indexer.runSpindexer(0.50);
                    indexer.runTransfer(0.50);
                    //                    indexer.runTransferBack(0.50);
                  },
                  indexer));
    }
    /** Switches from idle state when the indexer is requested and defines transition conditions */
    idleState.switchTo(churnState).when(() -> indexer.isRequested(IndexerState.CHURN));
    idleState.switchTo(feedState).when(() -> indexer.isRequested(IndexerState.FEED));
    idleState.switchTo(forceState).when(() -> indexer.isRequested(IndexerState.FORCE));
    churnState.switchTo(feedState).when(() -> indexer.isRequested(IndexerState.FEED));
    churnState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    churnState
        .switchTo(forceState)
        .when(
            () -> {
              return indexer.isRequested(IndexerState.FORCE);
            });
    /** Defines transitions for feed and force states */
    feedState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    feedState.switchTo(churnState).when(() -> indexer.isRequested(IndexerState.CHURN));
    feedState.switchTo(forceState).when(() -> indexer.isRequested(IndexerState.FORCE));
    forceState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    forceState.switchTo(churnState).when(() -> indexer.isRequested(IndexerState.CHURN));
    stateMachine.setInitialState(idleState);

    if (indexer != null) {
      stateMachine.addRequirements(indexer);
    }
    indexer.setDefaultCommands(stateMachine);
  }

  // TODO: Adjust Button Inputs
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createLeftBumper().onTrue(toggleForceFeed());
    operator.createLeftBumper().onTrue(shouldForceCommand()).onFalse(shouldChurnCommand());
    // operator.createLeftBumper().onTrue(shouldForceCommand()).onFalse(shouldChurnCommand());
  }

  /** Configures new triggers */
  private void configureTriggerStates() {
    Trigger feedTrigger = new Trigger(() -> indexer.isRequested(IndexerState.FEED));
    Trigger forceTrigger = new Trigger(() -> indexer.isRequested(IndexerState.FORCE));
    Trigger idleTrigger = new Trigger(() -> indexer.isRequested(IndexerState.IDLE));
    Trigger churnTrigger = new Trigger(() -> indexer.isRequested(IndexerState.CHURN));
    feedTrigger.onTrue(feedStateCommand());
    forceTrigger.onTrue(forceStateCommand());
    idleTrigger.onTrue(idleStateCommand());
    churnTrigger.onTrue(churnStateCommand());
  }

  public void setupDefaultCommands() {
    indexer.setDefaultCommands(stateMachine);
  }

  /** defines command behavio for the force state stops the indexer and runs the transfer at 50% */
  public static Command forceStateCommand() {
    return Commands.runOnce(
        () -> {
          indexer.setCurrentState(IndexerState.FORCE);
          indexer.runSpindexer(Constants.Indexer.SPINDEXER_SPEED);
          indexer.runTransfer(Constants.Indexer.TRANSFER_SPEED);
          //          indexer.runTransferBack(0.50);
        },
        indexer);
  }

  /** defines command behavior for the churn state stops the indexer and runs the transfer at 25% */
  private static Command churnStateCommand() {
    return Commands.runOnce(
        () -> {
          indexer.setCurrentState(IndexerState.CHURN);
          indexer.runSpindexer(-0.1);
          indexer.runTransfer(Constants.Indexer.TRANSFER_CHURN);
          //          indexer.runTransferBack(0.25);
        },
        indexer);
  }

  /**
   * defines command behavior for the idle state stops all motors and sets the LED patters to
   * rainbow
   */
  private static Command idleStateCommand() {
    return Commands.runOnce(
        () -> {
          indexer.setCurrentState(IndexerState.IDLE);
          indexer.runSpindexer(0);
          indexer.runTransfer(0);
          //          indexer.runTransferBack(0);
          LEDStrip.changeSegmentPattern(ConfigConstants.ALL_LEDS, LEDStrip.getRainbowPattern(0));
        },
        indexer);
  }

  // run feed command when Launcher State is idle and Operator Right Bumper is
  // pressed
  private static Command feedStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.FEED);
              indexer.runSpindexer(Constants.Indexer.SPINDEXER_SPEED);
              indexer.runTransfer(Constants.Indexer.TRANSFER_SPEED);
              //              indexer.runTransferBack(1);
              LEDStrip.changeSegmentPattern(
                  ConfigConstants.ALL_LEDS, LEDStrip.getRainbowPattern(25));
            },
            indexer));
  }
  /** Requests the indexer to enter the idle state */
  public static Command shouldIdleCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.IDLE));
  }
  /** Requests the indexer to enter the churn state */
  public static Command shouldChurnCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.CHURN));
  }
  /** Requests the indexer to enter the feed state */
  public static Command shouldFeedCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.FEED));
  }

  public static Command shouldForceCommand() {
    return Commands.runOnce(
        () -> {
          indexer.setRequestedState(IndexerState.FORCE);
        });
  }

  public static Command toggleForceFeed() {
    return Commands.runOnce(
        () -> {
          if (indexer.isRequested(IndexerState.FEED)) {
            indexer.setRequestedState(IndexerState.CHURN);

          } else {
            indexer.setRequestedState(IndexerState.FEED);
          }
        });
  }
}
