package org.frc5010.common.sensors.camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import gg.questnav.questnav.protos.generated.Data.ProtobufQuestNavDeviceData;
import gg.questnav.questnav.protos.generated.Data.ProtobufQuestNavFrameData;
import gg.questnav.questnav.protos.wpilib.DeviceDataProto;
import gg.questnav.questnav.protos.wpilib.FrameDataProto;
import java.util.Random;
import java.util.function.Supplier;

/**
 * A fake Quest headset for simulation, ported from FRC5010/QuestNavImplementation.
 *
 * <p>It reads the simulated robot's ground-truth pose (e.g. from maple-sim), works out where a
 * headset mounted at {@code robotToQuest} would be, and publishes that on the same NetworkTables
 * topics the real headset uses ({@code QuestNav/frameData}, {@code deviceData}, {@code version}).
 * The real {@code QuestNav} library and {@link QuestNavInterface} therefore run unmodified against
 * it.
 *
 * <p>Error sources you can dial in from the dashboard (all under {@code QuestNavSim/}), all off by
 * default so the headset starts out perfect:
 *
 * <ul>
 *   <li>{@code Tracking}: false makes frames report tracking lost.
 *   <li>{@code LatencyMs}: back-dates each frame's timestamp. NetworkTables drops a value stamped
 *       older than the previous one, so lowering this while running briefly loses frames.
 *   <li>{@code NoiseTranslationMeters} / {@code NoiseHeadingDeg}: Gaussian noise per frame.
 *   <li>{@code OriginX}, {@code OriginY}, {@code OriginDeg}: where the headset's own world origin
 *       sits in field coordinates. Zero means the two frames are aligned.
 * </ul>
 *
 * It also publishes {@code QuestNavSim/TruePose} and {@code QuestNavSim/QuestPose} for viewing.
 */
public class QuestNavSim implements AutoCloseable {
  private static final String ROOT = "QuestNavSim/";
  private static final int BATTERY_PERCENT = 100;
  // NT clients normally send only the latest value every 100 ms. The real headset streams every
  // frame, and the library treats data older than 120 ms as "disconnected", so match that.
  private static final PubSubOption[] FAST_SEND_OPTIONS = {
    PubSubOption.sendAll(true), PubSubOption.periodic(0.005)
  };

  // The real headset is a remote NT client of the robot's NT server, and the library reads the
  // server-assigned timestamp of each frame. Values published directly on the robot's own
  // instance never get one (it stays at 1 us), so the fake headset must be a separate client.
  private final NetworkTableInstance headsetInstance = connectHeadsetClient();
  private final NetworkTable questTable = headsetInstance.getTable("QuestNav");
  private final ProtobufPublisher<ProtobufQuestNavFrameData> framePublisher =
      questTable.getProtobufTopic("frameData", new FrameDataProto()).publish(FAST_SEND_OPTIONS);
  private final ProtobufPublisher<ProtobufQuestNavDeviceData> devicePublisher =
      questTable.getProtobufTopic("deviceData", new DeviceDataProto()).publish(FAST_SEND_OPTIONS);
  private final StringPublisher versionPublisher = questTable.getStringTopic("version").publish();
  private final StructPublisher<Pose2d> truePosePublisher =
      NetworkTableInstance.getDefault().getStructTopic(ROOT + "TruePose", Pose2d.struct).publish();
  private final StructPublisher<Pose3d> questPosePublisher =
      NetworkTableInstance.getDefault().getStructTopic(ROOT + "QuestPose", Pose3d.struct).publish();

  private final ProtobufQuestNavFrameData frame = ProtobufQuestNavFrameData.newInstance();
  private final ProtobufQuestNavDeviceData device = ProtobufQuestNavDeviceData.newInstance();
  private final Random random = new Random(0);

  private final Supplier<Pose2d> trueRobotPoseSupplier;
  private Transform3d robotToQuest;
  private int frameCount = 0;
  private int trackingLostCounter = 0;
  private boolean wasTracking = true;

  /**
   * Creates a simulated headset.
   *
   * @param robotToQuest where the headset is physically mounted (may differ from what the robot
   *     code assumes, to test calibration)
   * @param libVersion the QuestNav library version, so the library sees a matching "headset app"
   * @param trueRobotPoseSupplier the simulated robot's ground-truth field pose
   */
  public QuestNavSim(
      Transform3d robotToQuest, String libVersion, Supplier<Pose2d> trueRobotPoseSupplier) {
    this.robotToQuest = robotToQuest;
    this.trueRobotPoseSupplier = trueRobotPoseSupplier;
    versionPublisher.set(libVersion);

    SmartDashboard.setDefaultBoolean(ROOT + "Tracking", true);
    SmartDashboard.setDefaultNumber(ROOT + "LatencyMs", 0.0);
    SmartDashboard.setDefaultNumber(ROOT + "NoiseTranslationMeters", 0.0);
    SmartDashboard.setDefaultNumber(ROOT + "NoiseHeadingDeg", 0.0);
    SmartDashboard.setDefaultNumber(ROOT + "OriginX", 0.0);
    SmartDashboard.setDefaultNumber(ROOT + "OriginY", 0.0);
    SmartDashboard.setDefaultNumber(ROOT + "OriginDeg", 0.0);
  }

  private static NetworkTableInstance connectHeadsetClient() {
    NetworkTableInstance robot = NetworkTableInstance.getDefault();
    // `simulateJava` already runs a server; plain unit tests do not.
    if (!robot.getNetworkMode().contains(NetworkTableInstance.NetworkMode.kServer)) {
      robot.startServer(
          "", "", NetworkTableInstance.kDefaultPort3, NetworkTableInstance.kDefaultPort4);
    }
    NetworkTableInstance headset = NetworkTableInstance.create();
    headset.startClient4("QuestNavSim");
    headset.setServer("127.0.0.1", NetworkTableInstance.kDefaultPort4);
    return headset;
  }

  /**
   * Returns whether the fake headset has connected to the robot's NetworkTables server.
   *
   * @return true once connected
   */
  public boolean isHeadsetConnected() {
    return headsetInstance.isConnected();
  }

  @Override
  public void close() {
    headsetInstance.close();
  }

  /**
   * Returns where the robot really is, as opposed to what the pose estimator thinks.
   *
   * @return the ground-truth robot pose
   */
  public Pose2d getTrueRobotPose() {
    return trueRobotPoseSupplier.get();
  }

  /**
   * Changes where the simulated headset is mounted, e.g. to test offset calibration.
   *
   * @param robotToQuest the new physical mount transform
   */
  public void setMountTransform(Transform3d robotToQuest) {
    this.robotToQuest = robotToQuest;
  }

  /** Publishes one frame of headset data for the current ground-truth pose. */
  public void update() {
    boolean tracking = SmartDashboard.getBoolean(ROOT + "Tracking", true);
    if (wasTracking && !tracking) {
      trackingLostCounter++;
    }
    wasTracking = tracking;

    Pose2d origin =
        new Pose2d(
            SmartDashboard.getNumber(ROOT + "OriginX", 0.0),
            SmartDashboard.getNumber(ROOT + "OriginY", 0.0),
            Rotation2d.fromDegrees(SmartDashboard.getNumber(ROOT + "OriginDeg", 0.0)));

    // Headset pose in the field, then expressed in the headset's own world frame.
    Pose2d trueRobotPose = getTrueRobotPose();
    Pose3d questInField = new Pose3d(trueRobotPose).transformBy(robotToQuest);
    Pose3d questPose = questInField.relativeTo(new Pose3d(origin));

    double noiseTranslation = SmartDashboard.getNumber(ROOT + "NoiseTranslationMeters", 0.0);
    double noiseHeadingRad =
        Math.toRadians(SmartDashboard.getNumber(ROOT + "NoiseHeadingDeg", 0.0));
    if (noiseTranslation != 0.0 || noiseHeadingRad != 0.0) {
      questPose =
          questPose.transformBy(
              new Transform3d(
                  random.nextGaussian() * noiseTranslation,
                  random.nextGaussian() * noiseTranslation,
                  0.0,
                  new Rotation3d(0.0, 0.0, random.nextGaussian() * noiseHeadingRad)));
    }

    long nowMicros = RobotController.getFPGATime();
    long sampleMicros =
        nowMicros - (long) (SmartDashboard.getNumber(ROOT + "LatencyMs", 0.0) * 1000.0);

    frame.setTimestamp(sampleMicros / 1e6).setFrameCount(frameCount++).setIsTracking(tracking);
    Pose3d.proto.pack(frame.getMutablePose3D(), questPose);
    framePublisher.set(frame, sampleMicros);

    device.setBatteryPercent(BATTERY_PERCENT).setTrackingLostCounter(trackingLostCounter);
    devicePublisher.set(device, nowMicros);

    truePosePublisher.set(trueRobotPose);
    questPosePublisher.set(questPose);
  }
}
