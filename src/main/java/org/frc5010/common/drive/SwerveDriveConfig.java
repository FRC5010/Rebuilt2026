// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Map;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.devices.DeviceConfigReader;
import org.frc5010.common.config.json.devices.DrivetrainConstantsJson;
import org.frc5010.common.config.json.devices.DrivetrainConstantsJson.ModuleConfigJson;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/*Configuration constants for the swerve drive */
public class SwerveDriveConfig extends DrivetrainConfig
{
    /* The gear ratio of the steer motor */
    protected double steerGearRatio;
    /* The configuration for the steer motor controller */
    protected SmartMotorControllerConfig steerMotorControlConfig;
    /**
     * The ratio of the drive motor to the steer motor. Every 1 rotation of the azimuth results in
     * kCoupleRatio drive motor turns;
     */
    protected double coupleRatio;
    /* The steer inertia constant */
    protected MomentOfInertia steerInertia;
    /**
     * The module configurations, mapped by module name which should be frontLeft, frontRight,
     * backLeft, backRight
     */
    protected Map<String, SwerveModuleConfig> modules;

    protected SwerveDriveConfig() {}

    public SwerveDriveConfig(SwerveDriveConfigBuilder builder)
    {
        super(builder);
        this.wheelBase               = builder.wheelBase;
        this.coupleRatio             = builder.coupleRatio;
        this.steerGearRatio          = builder.steerGearRatio;
        this.steerMotorControlConfig = builder.steerMotorControlConfig;
        this.steerInertia            = builder.steerInertia;
        this.modules                 = builder.modules;
    }

    public static SwerveDriveConfigBuilder builder()
    {
        return new SwerveDriveConfigBuilder();
    }

    public static class SwerveDriveConfigBuilder extends DrivetrainConfig.DrivetrainConfigBuilder
    {
        protected Distance wheelBase;
        protected double steerGearRatio;
        protected SmartMotorControllerConfig steerMotorControlConfig;
        protected double coupleRatio;
        protected MomentOfInertia steerInertia;
        protected Map<String, SwerveModuleConfig> modules;

        public SwerveDriveConfigBuilder() {}

        public SwerveDriveConfig build(DrivetrainConstantsJson constants, SubsystemBase subsystem)
        {
            super.build(constants, subsystem);

            withSteerGearRatio(new MechanismGearing(GearBox.fromStages(constants.steerGearRatio))
                                   .getRotorToMechanismRatio());
            withCoupleRatio(constants.coupleRatio);
            withSteerInertia(UnitsParser.parseMomentOfInertia(constants.steerInertia));
            withWheelBase(UnitsParser.parseDistance(constants.wheelBase));

            for (ModuleConfigJson moduleConfig : constants.modules.values())
            {
                modules.put(
                    moduleConfig.toString(),
                    new SwerveModuleConfig()
                        .withDriveMotorInverted(moduleConfig.driveMotorSetup.inverted)
                        .withSteerMotorInverted(moduleConfig.steerMotorSetup.inverted)
                        .withEncoderInverted(moduleConfig.encoderInverted)
                        .withEncoderId(moduleConfig.encoderId)
                        .withAbsoluteOffset(UnitsParser.parseAngle(moduleConfig.absoluteOffset))
                        .withDriveMotor(
                            DeviceConfigReader
                                .getSmartMotor(moduleConfig.driveMotorSetup.controllerType,
                                               moduleConfig.driveMotorSetup.motorType,
                                               moduleConfig.driveMotorSetup.canId,
                                               new SmartMotorControllerConfig(subsystem)
                                                   .withControlMode(ControlMode.CLOSED_LOOP)
                                                   .withClosedLoopController(
                                                       constants.driveMotorControl.feedBack.p,
                                                       constants.driveMotorControl.feedBack.i,
                                                       constants.driveMotorControl.feedBack.d)
                                                   .withTelemetry(moduleConfig.driveMotorSetup.name
                                                                      + " DriveMotor",
                                                                  TelemetryVerbosity.MID)
                                                   .withFeedforward(new SimpleMotorFeedforward(
                                                       constants.driveMotorControl.feedForward.s,
                                                       constants.driveMotorControl.feedForward.v,
                                                       constants.driveMotorControl.feedForward.a)),
                                               canbus)
                                .orElse(null))
                        .withSteerMotor(
                            DeviceConfigReader
                                .getSmartMotor(moduleConfig.steerMotorSetup.controllerType,
                                               moduleConfig.steerMotorSetup.motorType,
                                               moduleConfig.steerMotorSetup.canId,
                                               new SmartMotorControllerConfig(subsystem)
                                                   .withControlMode(ControlMode.CLOSED_LOOP)
                                                   .withClosedLoopController(
                                                       constants.steerMotorControl.feedBack.p,
                                                       constants.steerMotorControl.feedBack.i,
                                                       constants.steerMotorControl.feedBack.d)
                                                   .withTelemetry(moduleConfig.steerMotorSetup.name
                                                                      + " SteerMotor",
                                                                  TelemetryVerbosity.MID)
                                                   .withFeedforward(new SimpleMotorFeedforward(
                                                       constants.steerMotorControl.feedForward.s,
                                                       constants.steerMotorControl.feedForward.v,
                                                       constants.steerMotorControl.feedForward.a)),
                                               canbus)
                                .orElse(null)));
            }

            return new SwerveDriveConfig(this);
        }

        public SwerveDriveConfigBuilder withSteerGearRatio(double steerGearRatio)
        {
            this.steerGearRatio = steerGearRatio;
            return this;
        }

        public SwerveDriveConfigBuilder withSteerMotorControlConfig(
            SmartMotorControllerConfig steerMotorControlConfig)
        {
            this.steerMotorControlConfig = steerMotorControlConfig;
            return this;
        }

        public SwerveDriveConfigBuilder withCoupleRatio(double coupleRatio)
        {
            this.coupleRatio = coupleRatio;
            return this;
        }

        public SwerveDriveConfigBuilder withSteerInertia(MomentOfInertia steerInertia)
        {
            this.steerInertia = steerInertia;
            return this;
        }

        public SwerveDriveConfigBuilder withModules(Map<String, SwerveModuleConfig> modules)
        {
            this.modules = modules;
            return this;
        }

        /**
         * Set the wheel base of the drivetrain.
         *
         * <p>This value should be set to the distance between the centers of the left and right
         * wheels on the drivetrain in meters. This value is used by the drivetrain to calculate its
         * linear velocity.
         *
         * @param wheelBase the wheel base of the drivetrain in meters
         * @return this builder
         */
        public DrivetrainConfigBuilder withWheelBase(Distance wheelBase)
        {
            this.wheelBase = wheelBase;
            return this;
        }
    }

    public static class SwerveModuleConfig
    {
        protected SmartMotorController driveMotor;
        protected SmartMotorController steerMotor;
        protected int encoderId;

        protected Angle absoluteOffset;
        protected boolean steerMotorInverted;
        protected boolean driveMotorInverted;
        protected boolean encoderInverted;

        public SwerveModuleConfig() {}

        public SwerveModuleConfig withDriveMotor(SmartMotorController driveMotor)
        {
            this.driveMotor = driveMotor;
            return this;
        }

        public SwerveModuleConfig withSteerMotor(SmartMotorController steerMotor)
        {
            this.steerMotor = steerMotor;
            return this;
        }

        public SwerveModuleConfig withEncoderId(int encoderId)
        {
            this.encoderId = encoderId;
            return this;
        }

        public SwerveModuleConfig withAbsoluteOffset(Angle absoluteOffset)
        {
            this.absoluteOffset = absoluteOffset;
            return this;
        }

        public SwerveModuleConfig withSteerMotorInverted(boolean steerMotorInverted)
        {
            this.steerMotorInverted = steerMotorInverted;
            return this;
        }

        public SwerveModuleConfig withDriveMotorInverted(boolean driveMotorInverted)
        {
            this.driveMotorInverted = driveMotorInverted;
            return this;
        }

        public SwerveModuleConfig withEncoderInverted(boolean encoderInverted)
        {
            this.encoderInverted = encoderInverted;
            return this;
        }
    }

    public Map<String, SwerveModuleConfig> getModules()
    {
        return modules;
    }

    public double getSteerGearRatio()
    {
        return steerGearRatio;
    }

    public SmartMotorControllerConfig getSteerMotorControlConfig()
    {
        return steerMotorControlConfig;
    }

    public double getCoupleRatio()
    {
        return coupleRatio;
    }

    public MomentOfInertia getSteerInertia()
    {
        return steerInertia;
    }

    public Distance getWheelBase()
    {
        return wheelBase;
    }
}
