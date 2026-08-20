package frc.robot.rebuilt.commands;

import frc.robot.lib.BLine.FollowPath;

public class B_LineAutoCommands {

  public void configureBlineAutoCommands() {
    // INTAKE
    FollowPath.registerEventTrigger("intakeIntake", IntakeCommands.shouldIntaking());
    FollowPath.registerEventTrigger("intakeRetracted", IntakeCommands.shouldIntaking());
    FollowPath.registerEventTrigger("intakeRetracting", IntakeCommands.shouldIntaking());
    // LAUNCHER
    FollowPath.registerEventTrigger("launcherPrep", LauncherCommands.shouldPrepCommand());
    FollowPath.registerEventTrigger("launcherPreset", LauncherCommands.shouldPresetCommand());
    FollowPath.registerEventTrigger("launcherLow", LauncherCommands.shouldLowCommand());
    FollowPath.registerEventTrigger("launcherIdle", LauncherCommands.shouldIdleCommand());
    // PRESET
    FollowPath.registerEventTrigger("iForcePreset", IndexerCommands.shouldForceCommand());
    FollowPath.registerEventTrigger("hubPreset", LauncherCommands.leftCornerPresetStateCommand());
    FollowPath.registerEventTrigger("towerPreset", LauncherCommands.towerPresetStateCommand());
    FollowPath.registerEventTrigger("towerForwardPreset", LauncherCommands.turretForwardPresetStateCommand());
    FollowPath.registerEventTrigger("WaitUntilIntaking", IntakeCommands.waitUntilIntaking());
    // INDEXER
    FollowPath.registerEventTrigger("indexerChurn", IndexerCommands.shouldChurnCommand());
    FollowPath.registerEventTrigger("indexerIdle", IndexerCommands.shouldIdleCommand());
    FollowPath.registerEventTrigger("indexerFeed", IndexerCommands.shouldFeedCommand());
  }
}
