package org.frc5010.common.sensors.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.pose.PoseProvider;

/** Add your docs here. */
public class QuestNavInterface implements PoseProvider {

  private String networkTableRoot = "questnav";
  private Supplier<ChassisSpeeds> robotVelocity = null;
  private Transform3d robotToQuest;
  private QuestNav questNav;
  private boolean initializedPosition = false;
  private PoseFrame latestPoseFrame = null;

  private ChassisSpeeds velocity;
  private Pose3d previousPose;
  private double previousTime;
  private static boolean hasHardReset = false;
  private static boolean initialReset = false;

  // I guess this works
  private static Pose2d latestPoseState = null;

  private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
  private int _calculatedOffsetToRobotCenterCount = 0;

  public QuestNavInterface(Transform3d robotToQuest) {
    super();
    this.robotToQuest = robotToQuest;
    this.questNav = new QuestNav();
  }

  public QuestNavInterface(Transform3d robotToQuest, String networkTableRoot) {
    super();
    this.robotToQuest = robotToQuest;
    this.networkTableRoot = networkTableRoot;
    this.questNav = new QuestNav();
  }

  private Pose3d getRobotPoseFromQuestPose(Pose3d questPose) {
    return questPose.transformBy(robotToQuest.inverse());
  }

  private Pose3d getQuestPoseFromRobotPose(Pose3d robotPose) {
    return robotPose.transformBy(robotToQuest);
  }

  private Pose3d getLatestQuestPose() {
    if (latestPoseFrame == null) {
      return null;
    }
    return latestPoseFrame.questPose3d();
  }

  public void withRobotSpeedSupplier(Supplier<ChassisSpeeds> robotSpeed) {
    robotVelocity = robotSpeed;
  }

  public Optional<Pose3d> getRobotPose() {
    if (RobotBase.isReal()) {
      if (latestPoseFrame == null) {
        return Optional.empty();
      }
      Pose3d pose = getRobotPoseFromQuestPose(getLatestQuestPose());
      return Optional.of(pose);
    } else {
      return Optional.empty();
    }
  }

  public Rotation3d getRotation() {
    if (latestPoseFrame == null) {
      return new Rotation3d();
    }
    return getRobotPoseFromQuestPose(getLatestQuestPose()).getRotation();
  }

  public Translation3d getPosition() {
    if (latestPoseFrame == null) {
      return new Translation3d();
    }
    return getRobotPoseFromQuestPose(getLatestQuestPose()).getTranslation();
  }

  private void updateObservations() {
    PoseFrame[] unreadQuestFrames = questNav.getAllUnreadPoseFrames();

    // Guard against empty frame array — an empty result (e.g. transient tracking loss)
    // previously caused an ArrayIndexOutOfBoundsException that silenced ALL providers.
    if (unreadQuestFrames.length == 0) {
      input.connected = false;
      input.poseObservations = new PoseObservation[0];
      return;
    }

    latestPoseFrame = unreadQuestFrames[unreadQuestFrames.length - 1];
    List<PoseObservation> observations = new ArrayList<>();

    for (PoseFrame frame : unreadQuestFrames) {
      Pose3d questPose = frame.questPose3d();
      Pose3d robotPose = getRobotPoseFromQuestPose(questPose);
      double captureTime = frame.dataTimestamp();
      observations.add(
          new PoseObservation(
              captureTime,
              robotPose,
              0,
              0,
              0,
              0.0, // effectiveSpan (N/A for Quest)
              PoseObservationType.ENVIRONMENT_BASED,
              ProviderType.ENVIRONMENT_BASED,
              PnpMethod.VISUAL_ODOMETRY));
    }
    input.connected = isActive();
    // Save pose observations to inputs object
    input.poseObservations = new PoseObservation[observations.size()];
    for (int i = 0; i < observations.size(); i++) {
      input.poseObservations[i] = observations.get(i);
    }
  }

  @Override
  public Matrix<N3, N1> getStdDeviations(PoseObservation observation) {
    double calib = getConfidence();

    // While disabled, give Quest a much higher std dev so that AprilTag cameras
    // (FIELD_BASED) can dominate the estimator and properly initialize the pose
    // for autonomous. Without this, Quest's 0.05 std dev drowns out AprilTag readings.
    if (DriverStation.isDisabled()) {
      calib = 2.0;
    } else if (null != robotVelocity) {
      Translation2d questVelVector =
          new Translation2d(getVelocity().vxMetersPerSecond, getVelocity().vyMetersPerSecond);
      Translation2d robotVelVector =
          new Translation2d(
              robotVelocity.get().vxMetersPerSecond, robotVelocity.get().vyMetersPerSecond);
      if (Math.abs(questVelVector.getNorm() - robotVelVector.getNorm()) > 1.0) {
        calib = 10;
      }
    }
    return VecBuilder.fill(calib, calib, calib * 0.2);
  }

  public double getConfidence() {
    if (RobotBase.isReal()) {
      return 0.05;
    } else {
      return Double.MAX_VALUE;
    }
  }

  @Override
  public boolean isConnected() {
    return questNav.isConnected();
  }

  public boolean isActive() {
    boolean simulation = RobotBase.isSimulation();
    // boolean disabled = DriverStation.isDisabled();

    boolean isActive =
        questNav.isConnected()
            && (questNav.getFrameCount().orElse(0) > 0 && !simulation)
            && questNav.isTracking();

    return isActive;
  }

  public void resetPose(Pose3d pose) {
    if (isConnected()) {
      Pose3d questPose = getQuestPoseFromRobotPose(pose);
      questNav.setPose(questPose);
      initializedPosition = true;
    }
  }

  @Override
  public ProviderType getType() {
    return ProviderType.ENVIRONMENT_BASED;
  }

  public int fiducialId() {
    return 0;
  }

  private void updateVelocity() {
    return; // Implement if needed.
  }

  public ChassisSpeeds getVelocity() {
    if (null != velocity) {
      return velocity;
    }
    return new ChassisSpeeds();
  }

  @Override
  public void update() {
    if (RobotBase.isReal()) {
      questNav.commandPeriodic();
      updateVelocity();
      updateObservations();
      SmartDashboard.putBoolean(networkTableRoot + "/Reset Pose", false);
      SmartDashboard.putBoolean("QUEST Connected", isConnected());
      SmartDashboard.putBoolean("QUEST Active", isActive());

      Pose2d currPose = getRobotPose().orElse(new Pose3d()).toPose2d();
      latestPoseState = currPose;
      SmartDashboard.putNumberArray(
          networkTableRoot + "/Quest POSE Update",
          new double[] {currPose.getX(), currPose.getY(), currPose.getRotation().getDegrees()});

      ChassisSpeeds velocity = getVelocity();
      SmartDashboard.putNumberArray(
          networkTableRoot + "/Velocity",
          new double[] {
            velocity.vxMetersPerSecond, velocity.vyMetersPerSecond, velocity.omegaRadiansPerSecond
          });
    }
    logInput(networkTableRoot);
  }

  private Translation2d calculateOffsetToRobotCenter() {
    if (null == latestPoseState) {
      return new Translation2d();
    }

    Pose2d currentPose2d = latestPoseState;

    Rotation2d angle = currentPose2d.getRotation();
    Translation2d displacement = currentPose2d.getTranslation();

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / (2 * (1 - angle.getCos()));
    double y =
        ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / (2 * (1 - angle.getCos()));

    return new Translation2d(x, y);
  }

  public Command determineOffsetToRobotCenter(GenericDrivetrain drivetrain) {
    return Commands.repeatingSequence(
            Commands.run(
                    () -> {
                      SmartDashboard.putNumber("QUEST POSE", getPosition().getX());
                      drivetrain.drive(new ChassisSpeeds(0, 0, 0.314));
                    },
                    drivetrain)
                .withTimeout(3.0),
            Commands.runOnce(
                () -> {
                  // Update current offset
                  Translation2d offset = calculateOffsetToRobotCenter();

                  _calculatedOffsetToRobotCenter =
                      _calculatedOffsetToRobotCenter
                          .times(
                              (double) _calculatedOffsetToRobotCenterCount
                                  / (_calculatedOffsetToRobotCenterCount + 1))
                          .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
                  _calculatedOffsetToRobotCenterCount++;

                  SmartDashboard.putNumberArray(
                      networkTableRoot + "/Quest Calculated Offset to Robot Center",
                      new double[] {
                        _calculatedOffsetToRobotCenter.getX(), _calculatedOffsetToRobotCenter.getY()
                      });
                }))
        .beforeStarting(
            Commands.runOnce(() -> resetPose(new Pose3d())).andThen(Commands.waitSeconds(1)));
  }
}
