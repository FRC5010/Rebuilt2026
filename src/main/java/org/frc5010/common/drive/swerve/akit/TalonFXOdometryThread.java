// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package org.frc5010.common.drive.swerve.akit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.frc5010.common.drive.swerve.AkitSwerveConfig;
import org.frc5010.common.drive.swerve.SwerveDriveFunctions;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
public class TalonFXOdometryThread extends OdometryThread {
  private final Lock signalsLock =
      new ReentrantLock(); // Prevents conflicts when registering signals
  private BaseStatusSignal[] phoenixSignals = new BaseStatusSignal[0];
  private final List<Queue<Double>> phoenixQueues = new ArrayList<>();
  AkitSwerveConfig config;

  private static boolean isCANFD;
  private static TalonFXOdometryThread instance = null;

  public static TalonFXOdometryThread getInstance() {
    return instance;
  }

  public static void createInstance(AkitSwerveConfig config) {
    if (instance == null) {
      instance = new TalonFXOdometryThread(config);
    }
  }

  private TalonFXOdometryThread(AkitSwerveConfig config) {
    super("TalonFXOdometryThread");
    this.config = config;
    isCANFD = config.getCANBus().isNetworkFD();
    commonInstance = this;
  }

  @Override
  public void start() {
    if (!timestampQueues.isEmpty() && RobotBase.isReal()) {
      super.start();
    }
  }

  /** Registers a Phoenix signal to be read from the thread. */
  public Queue<Double> registerSignal(StatusSignal<Angle> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    signalsLock.lock();
    SwerveDriveFunctions.odometryLock.lock();
    try {
      BaseStatusSignal[] newSignals = new BaseStatusSignal[phoenixSignals.length + 1];
      System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.length);
      newSignals[phoenixSignals.length] = signal;
      phoenixSignals = newSignals;
      phoenixQueues.add(queue);
    } finally {
      signalsLock.unlock();
      SwerveDriveFunctions.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    SwerveDriveFunctions.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      SwerveDriveFunctions.odometryLock.unlock();
    }
    return queue;
  }

  @Override
  public void runThreadLogic() {
    // Wait for updates from all signals
    signalsLock.lock();
    try {
      if (isCANFD && phoenixSignals.length > 0) {
        BaseStatusSignal.waitForAll(2.0 / config.ODOMETRY_FREQUENCY, phoenixSignals);
      } else {
        // "waitForAll" does not support blocking on multiple signals with a bus
        // that is not CAN FD, regardless of Pro licensing. No reasoning for this
        // behavior is provided by the documentation.
        Thread.sleep((long) (1000.0 / config.ODOMETRY_FREQUENCY));
        if (phoenixSignals.length > 0) BaseStatusSignal.refreshAll(phoenixSignals);
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      signalsLock.unlock();
    }

    // Save new data to queues
    SwerveDriveFunctions.odometryLock.lock();
    try {
      // Sample timestamp is current FPGA time minus average CAN latency
      // Default timestamps from Phoenix are NOT compatible with
      // FPGA timestamps, this solution is imperfect but close
      double timestamp = RobotController.getFPGATime() / 1e6;
      double totalLatency = 0.0;
      for (BaseStatusSignal signal : phoenixSignals) {
        totalLatency += signal.getTimestamp().getLatency();
      }
      if (phoenixSignals.length > 0) {
        timestamp -= totalLatency / phoenixSignals.length;
      }

      // Add new samples to queues
      for (int i = 0; i < phoenixSignals.length; i++) {
        phoenixQueues.get(i).offer(phoenixSignals[i].getValueAsDouble());
      }
      for (int i = 0; i < genericSignals.size(); i++) {
        genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
      }
      for (int i = 0; i < timestampQueues.size(); i++) {
        timestampQueues.get(i).offer(timestamp);
      }
    } finally {
      SwerveDriveFunctions.odometryLock.unlock();
    }
  }
}
