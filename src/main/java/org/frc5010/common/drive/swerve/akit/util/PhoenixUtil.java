// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package org.frc5010.common.drive.swerve.akit.util;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedMotorController;

public final class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private static int instances = 0;
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = instances++;

      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  public static class TalonFXMotorControllerWithRemoteCancoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);

      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
    }

    return odometryTimeStamps;
  }

  /**
   * Regulates a {@link SwerveModuleConstants} object for simulation. If running on a real robot,
   * the input object is returned unchanged. Otherwise, simulation-specific adjustments are made:
   * disable encoder offsets, disable motor/encoder inversions, switch closed-loop output to
   * Voltage, install sim-tuned Voltage PID gains for both steer and drive, and adjust friction
   * voltages and steer inertia.
   *
   * <p>Real-robot control is {@code TorqueCurrentFOC} with JSON-tuned amps/rotation gains. Those
   * gains have the wrong units for Voltage closed-loop and the wrong magnitudes for the maple-sim
   * motor model, so sim overrides both the output type and the gains. The AdvantageKit reference
   * Voltage steer gains (kP=70 V/rotation, kV=1.91 V·s/rotation) match this code path's lineage and
   * track cleanly under the maple-sim physics. Drive uses light Voltage gains where the kV
   * feedforward dominates.
   *
   * <p>The steer gear ratio is intentionally NOT overridden; it flows through from JSON so the
   * controller's mechanism-frame math matches what maple-sim is physically simulating.
   *
   * @param moduleConstants module constants to regulate
   * @return regulated module constants
   */
  public static SwerveModuleConstants regulateModuleConstantForSimulation(
      SwerveModuleConstants<?, ?, ?> moduleConstants) {
    // Skip regulation if running on a real robot
    if (RobotBase.isReal()) return moduleConstants;

    // Apply simulation-specific adjustments to module constants
    return moduleConstants
        // Disable encoder offsets
        .withEncoderOffset(0)
        // Disable motor inversions for drive and steer motors
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        // Disable CanCoder inversion
        .withEncoderInverted(false)
        // Use Voltage closed-loop in sim. JSON gains are TorqueCurrentFOC amps; reusing them under
        // Voltage would treat amps as volts and either saturate or wildly under-drive.
        .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
        // Sim-tuned steer Voltage gains (AdvantageKit reference values for sim).
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(70)
                .withKI(0)
                .withKD(4.5)
                .withKS(0)
                .withKV(1.91)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        // Sim-tuned drive Voltage gains. kV ≈ 12 V / 16.7 wheel rev/s (Kraken X60 free speed
        // 6000 RPM ÷ 6:1 reduction) so the feedforward alone delivers ~85% of max-speed authority.
        .withDriveMotorGains(
            new Slot0Configs().withKP(0.05).withKI(0).withKD(0).withKS(0.1).withKV(0.72).withKA(0))
        // Adjust friction voltages
        .withDriveFrictionVoltage(Volts.of(0.1))
        .withSteerFrictionVoltage(Volts.of(0.05))
        // Adjust steer inertia
        .withSteerInertia(KilogramSquareMeters.of(0.05));
  }
}
