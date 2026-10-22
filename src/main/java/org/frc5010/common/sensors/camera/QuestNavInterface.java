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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.drive.pose.PoseProvider;
import org.frc5010.common.drive.pose.VisionIOInputsAutoLogged;

/**
 * {@link PoseProvider} backed by a Meta Quest headset running QuestNav.
 *
 * <p>Headset poses are converted to robot poses through the configured robot-to-quest transform and
 * fed to the pose estimator as environment-based observations. While disabled the estimator keeps
 * soft-resetting this provider to its current pose, so the headset follows whatever start pose auto
 * (or a manual reset) sets. In simulation, {@link #withSimulation(Supplier)} attaches a {@link
 * QuestNavSim} so the same code path runs against a fake headset.
 */
public class QuestNavInterface implements PoseProvider {

  /** Per-instance inputs — each QuestNav has its own object */
  private final VisionIOInputsAutoLogged input = new VisionIOInputsAutoLogged();

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

  // Soft reset state — applied in code without sending a command to the headset
  // Raw pose at reset time and the pose it should map to; later poses are the motion since reset,
  // applied in the target pose's frame.
  private Pose3d softResetBase = new Pose3d();
  private Pose3d softResetTarget = new Pose3d();

  private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
  private int _calculatedOffsetToRobotCenterCount = 0;

  /** Reusable observation list — cleared each cycle to avoid per-cycle ArrayList allocation */
  private final List<PoseObservation> questObservations = new ArrayList<>();
  /** Grow-only output array — never shrunk to minimise GC pressure */
  private PoseObservation[] questObsArray = new PoseObservation[0];
  /** Pre-allocated stdDev matrix — mutated in getStdDeviations() to avoid per-call allocation */
  private final Matrix<N3, N1> stdDevMatrix = VecBuilder.fill(0.0, 0.0, 0.0);

  /** Translation std dev (meters) trusted from a tracking headset. */
  private static final double TRANSLATION_STD_DEV = 0.02;
  /** Heading std dev (radians, ~5 degrees) trusted from a tracking headset. */
  private static final double HEADING_STD_DEV = Math.toRadians(5);
  /** Minimum (1 - cos(angle)) for an offset sample to be usable; the estimate blows up near 0. */
  private static final double MIN_ONE_MINUS_COS = 0.1;

  /** Simulated headset; only present when running in simulation. */
  private QuestNavSim sim = null;

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

  @Override
  public VisionIOInputsAutoLogged getInput() {
    return input;
  }

  @Override
  public int getCameraIndex() {
    return 0;
  }

  private Pose3d getRobotPoseFromQuestPose(Pose3d questPose) {
    return questPose.transformBy(robotToQuest.inverse());
  }

  /** Maps a raw (hard-reset layer) robot pose through the current soft reset. */
  private Pose3d applySoftReset(Pose3d processedRobotPose) {
    return softResetTarget.transformBy(new Transform3d(softResetBase, processedRobotPose));
  }

  private Pose3d getQuestPoseFromRobotPose(Pose3d robotPose) {
    return robotPose.transformBy(robotToQuest);
  }

  public void withRobotSpeedSupplier(Supplier<ChassisSpeeds> robotSpeed) {
    robotVelocity = robotSpeed;
  }

  /**
   * Attaches a simulated headset that publishes frames for this interface to consume.
   *
   * @param trueRobotPoseSupplier the simulated robot's ground-truth field pose
   * @return this interface, for chaining
   */
  public QuestNavInterface withSimulation(Supplier<Pose2d> trueRobotPoseSupplier) {
    sim = new QuestNavSim(robotToQuest, questNav.getLibVersion(), trueRobotPoseSupplier);
    return this;
  }

  /**
   * Returns the simulated headset, present only when running in simulation.
   *
   * @return the simulated headset, if any
   */
  public Optional<QuestNavSim> getSim() {
    return Optional.ofNullable(sim);
  }

  /**
   * Returns the robot pose as computed directly from the QuestNav headset plus the robotToQuest
   * transform, with no soft-reset offset applied. This is the "hard-reset" pose layer.
   */
  private Pose3d getProcessedRobotPose() {
    if (latestPoseFrame == null) {
      return new Pose3d();
    }
    return getRobotPoseFromQuestPose(latestPoseFrame.questPose3d());
  }

  public Optional<Pose3d> getRobotPose() {
    if (latestPoseFrame == null) {
      return Optional.empty();
    }
    return Optional.of(applySoftReset(getProcessedRobotPose()));
  }

  public Rotation3d getRotation() {
    if (latestPoseFrame == null) {
      return new Rotation3d();
    }
    return applySoftReset(getProcessedRobotPose()).getRotation();
  }

  public Translation3d getPosition() {
    if (latestPoseFrame == null) {
      return new Translation3d();
    }
    return applySoftReset(getProcessedRobotPose()).getTranslation();
  }

  private void updateObservations() {
    PoseFrame[] unreadQuestFrames = questNav.getAllUnreadPoseFrames();
    if (unreadQuestFrames.length > 0) {
      latestPoseFrame = unreadQuestFrames[unreadQuestFrames.length - 1];
    }

    questObservations.clear();

    if (initializedPosition) {
      for (PoseFrame frame : unreadQuestFrames) {
        Pose3d robotPose = applySoftReset(getRobotPoseFromQuestPose(frame.questPose3d()));
        double captureTime = frame.dataTimestamp();
        questObservations.add(
            new PoseObservation(
                captureTime,
                robotPose,
                0,
                0,
                0,
                PoseObservationType.ENVIRONMENT_BASED,
                ProviderType.ENVIRONMENT_BASED));
      }
    }
    input.connected = isActive();
    // Save pose observations to inputs object using grow-only pre-allocated array
    int obsSize = questObservations.size();
    if (questObsArray.length != obsSize) {
      questObsArray = new PoseObservation[obsSize];
    }
    for (int i = 0; i < obsSize; i++) {
      questObsArray[i] = questObservations.get(i);
    }
    input.poseObservations = questObsArray;
  }

  @Override
  public Matrix<N3, N1> getStdDeviations(PoseObservation observation) {
    double calib = TRANSLATION_STD_DEV;
    double headingCalib = HEADING_STD_DEV;
    if (DriverStation.isDisabled()) {
      calib = 10000;
      headingCalib = 10000;
    }
    if (null != robotVelocity) {
      Translation2d questVelVector =
          new Translation2d(getVelocity().vxMetersPerSecond, getVelocity().vyMetersPerSecond);
      Translation2d robotVelVector =
          new Translation2d(
              robotVelocity.get().vxMetersPerSecond, robotVelocity.get().vyMetersPerSecond);
      if (Math.abs(questVelVector.getNorm() - robotVelVector.getNorm()) > 1.0) {
        calib = 10;
        headingCalib = 10;
      }
    }
    stdDevMatrix.set(0, 0, calib);
    stdDevMatrix.set(1, 0, calib);
    stdDevMatrix.set(2, 0, headingCalib);
    return stdDevMatrix;
  }

  @Override
  public boolean isConnected() {
    return questNav.isConnected();
  }

  public boolean isActive() {
    boolean isActive =
        questNav.isConnected() && questNav.getFrameCount().orElse(0) > 0 && questNav.isTracking();

    return isActive && initializedPosition;
  }

  /**
   * Performs a soft reset: recalculates the pose offset in code without sending any command to the
   * Quest headset. This is instantaneous and avoids the delay of a hard reset command. The
   * soft-reset transform is stored and applied on top of the raw headset pose in {@link
   * #getRobotPose()}, {@link #getPosition()}, and {@link #getRotation()}.
   */
  public void softReset(Pose3d pose) {
    softResetBase = getProcessedRobotPose();
    softResetTarget = pose;
    initializedPosition = true;
  }

  /**
   * Performs a hard reset: sends {@code questNav.setPose()} to the headset so its internal
   * coordinate origin is relocated. Intended to be called only once at the beginning of a match.
   * Subsequent resets during the match should use {@link #softReset(Pose3d)}.
   */
  public void hardReset(Pose3d pose) {
    if (isConnected()) {
      Pose3d questPose = getQuestPoseFromRobotPose(pose);
      questNav.setPose(questPose);
      // Clear any accumulated soft-reset so the hard reset is authoritative
      softResetBase = new Pose3d();
      softResetTarget = new Pose3d();
      initializedPosition = true;
      hasHardReset = initialReset;
      initialReset = true;
    }
  }

  public static Trigger hasHardReset() {
    return new Trigger(() -> hasHardReset);
  }

  @Override
  public void resetPose(Pose3d pose) {
    if (isConnected()) {
      SmartDashboard.putBoolean(networkTableRoot + "/Reset Pose", true);
      softReset(pose);
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
    // In replay there is no headset or sim; logged inputs are restored by logInput() below.
    if (RobotBase.isReal() || sim != null) {
      if (sim != null) {
        sim.update();
        SmartDashboard.putNumber(
            "QuestNavSim/EstimateErrorMeters",
            getRobotPose()
                .map(
                    p ->
                        p.toPose2d()
                            .getTranslation()
                            .getDistance(sim.getTrueRobotPose().getTranslation()))
                .orElse(Double.NaN));
      }
      questNav.commandPeriodic();
      updateVelocity();
      updateObservations();
      SmartDashboard.putBoolean(networkTableRoot + "/Reset Pose", false);
      SmartDashboard.putBoolean("QUEST Connected", isConnected());
      SmartDashboard.putBoolean("QUEST Active", isActive());
      SmartDashboard.putBoolean(networkTableRoot + "/Tracking", questNav.isTracking());
      SmartDashboard.putNumber(networkTableRoot + "/Latency", questNav.getLatency());
      questNav
          .getBatteryPercent()
          .ifPresent(b -> SmartDashboard.putNumber(networkTableRoot + "/Battery%", b));
      questNav
          .getTrackingLostCounter()
          .ifPresent(c -> SmartDashboard.putNumber(networkTableRoot + "/TrackingLostCount", c));

      Pose2d currPose = getRobotPose().orElse(new Pose3d()).toPose2d();
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

  /**
   * Offset of the tracked robot origin from the rotation center, from the pose change since the
   * last soft reset. Empty if there is no usable sample.
   */
  private Optional<Translation2d> calculateOffsetToRobotCenter() {
    if (!isConnected()) {
      return Optional.empty();
    }
    Optional<Pose2d> currentPose = getRobotPose().map(Pose3d::toPose2d);
    if (currentPose.isEmpty()) {
      return Optional.empty();
    }

    Rotation2d angle = currentPose.get().getRotation();
    Translation2d displacement = currentPose.get().getTranslation();

    double denominator = 2 * (1 - angle.getCos());
    if (denominator < 2 * MIN_ONE_MINUS_COS) {
      return Optional.empty();
    }

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / denominator;
    double y =
        ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / denominator;

    return Optional.of(new Translation2d(x, y));
  }

  /**
   * Spins the robot in place to measure where the headset sits relative to the rotation center.
   * Each iteration soft-resets the pose, spins ~1.5 rad, then takes one sample, so every sample
   * uses a well-conditioned angle. The running average is published to {@code <root>/Quest
   * Calculated Offset to Robot Center}. Bind with whileTrue to stop it.
   *
   * @param drivetrain the drivetrain to spin
   * @return the calibration command
   */
  public Command determineOffsetToRobotCenter(GenericDrivetrain drivetrain) {
    return Commands.repeatingSequence(
            Commands.runOnce(() -> resetPose(new Pose3d())),
            Commands.run(() -> drivetrain.drive(new ChassisSpeeds(0, 0, 0.5)), drivetrain)
                .withTimeout(3.0),
            Commands.runOnce(
                () ->
                    calculateOffsetToRobotCenter()
                        .ifPresent(
                            offset -> {
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
                                    _calculatedOffsetToRobotCenter.getX(),
                                    _calculatedOffsetToRobotCenter.getY()
                                  });
                            })))
        .beforeStarting(
            () -> {
              _calculatedOffsetToRobotCenter = new Translation2d();
              _calculatedOffsetToRobotCenterCount = 0;
            });
  }
}
