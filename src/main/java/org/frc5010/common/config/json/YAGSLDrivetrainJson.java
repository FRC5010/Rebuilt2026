// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.constants.MotorFeedFwdConstants;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.YAGSLSwerveDrivetrain;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

/** Parameters for a YAGSLSwerveDrivetrain */
public class YAGSLDrivetrainJson implements DrivetrainPropertiesJson
{
    /** The directory to read from */
    public String directory = "";

    /** The gear ratio of the turning motor */
    public double turningMotorGearRatio = 1.0;

    private Optional<GamePiecesJson> gamePiecesJson = Optional.empty();
    /**
     * The file names of the drive module's feed forward constants. These should match what are used
     * in the YAGSL config
     */
    public String[] driveModules;

    /** Starting pose of the robot */
    public Pose2dJson startingPose = new Pose2dJson();

    @Override
    public void readDrivetrainConfiguration(GenericRobot robot, File baseDirectory)
        throws IOException
    {
        SwerveConstants swerveConstants = new SwerveConstants(robot.getDrivetrainConstants());
        for (String driveModule : driveModules)
        {
            File moduleFile   = new File(baseDirectory, "drive_modules/" + driveModule);
            String moduleName = driveModule.substring(0, driveModule.indexOf(".json"));
            assert moduleFile.exists();
            YAGSLDriveModuleJson module =
                new ObjectMapper().readValue(moduleFile, YAGSLDriveModuleJson.class);
            MotorFeedFwdConstants feedFwdConstants =
                new MotorFeedFwdConstants(module.s, module.v, module.a);
            swerveConstants.getSwerveModuleConstants().addDriveMotorFF(moduleName,
                                                                       feedFwdConstants);
        }
        robot.setDrivetrainConstants(swerveConstants);

        if (RobotBase.isSimulation())
        {
            File fieldDirectory = new File(baseDirectory, "/field/");
            if (fieldDirectory.exists())
            {
                File gamePiecesFile = new File(fieldDirectory, "game_pieces.json");
                if (gamePiecesFile.exists())
                {
                    gamePiecesJson = Optional.ofNullable(
                        new ObjectMapper().readValue(gamePiecesFile, GamePiecesJson.class));
                }
            }
        }
    };

    @Override public void createDriveTrain(GenericRobot robot)
    {
        Pose2d startingPoseFromJson =
            new Pose2d(UnitsParser.parseDistance(startingPose.x).in(Meters),
                       UnitsParser.parseDistance(startingPose.y).in(Meters),
                       new Rotation2d(UnitsParser.parseAngle(startingPose.rotation).in(Degrees)));
        YAGSLSwerveDrivetrain yagsl = new YAGSLSwerveDrivetrain(
            robot.getDrivetrainConstants(), turningMotorGearRatio, directory, startingPoseFromJson);
        GenericSwerveDrivetrain drivetrain = new GenericSwerveDrivetrain(
            new LoggedMechanism2d(RobotConstantsDef.robotVisualH, RobotConstantsDef.robotVisualV),
            robot.getDrivetrainConstants(), yagsl);
        robot.addSubsystem(ConfigConstants.DRIVETRAIN, drivetrain);
        robot.setPoseSupplier(() -> drivetrain.getPoseEstimator().getCurrentPose());
        robot.setSimulatedPoseSupplier(() -> yagsl.getSimPose());
        gamePiecesJson.ifPresent(it -> it.createGamePieces(drivetrain));
    }
}
