// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.drive;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.devices.DrivetrainConstantsJson;
import org.frc5010.common.sensors.gyro.GenericGyro;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorControllerConfig;

/** A class containing constants for the drivetrain */
public class DrivetrainConfig
{
    /** The track width */
    protected Distance trackWidth;
    /** The wheel base */
    protected Distance wheelBase;
    /** The width of the drivetrain including the bumpers */
    protected Distance bumperFrameWidth;
    /** The length of the drivetrain including the bumpers */
    protected Distance bumperFrameLength;
    /** The wheel diameter */
    protected Distance wheelDiameter;
    /** The maximum drive speed */
    protected LinearVelocity maxDriveSpeed;
    /** The drive gear ratio */
    protected double driveGearRatio;
    /** The current draw at which the wheels slip */
    protected Current slipCurrent;
    /** Whether to invert the left side */
    protected boolean invertLeftSide;
    /** Whether to invert the right side */
    protected boolean invertRightSide;
    /** The drive inertia */
    protected MomentOfInertia driveInertia;
    /** The robot mass */
    protected Mass robotMass;
    /** The drivetrain type */
    protected String drivetrainType;
    /** The gyro used by the drivetrain */
    protected GenericGyro gyro;
    /** The drive motor controller configuration */
    protected SmartMotorControllerConfig driveMotorControlConfig;
    /** The CAN bus for the drivetrain */
    protected String canbus;

    /**
     * Returns a new drivetrain configuration builder.
     *
     * @return a new drivetrain configuration builder
     */
    public static DrivetrainConfigBuilder builder()
    {
        return new DrivetrainConfigBuilder();
    }

    protected DrivetrainConfig() {}

    protected DrivetrainConfig(DrivetrainConfigBuilder builder)
    {
        this.trackWidth              = builder.trackWidth;
        this.wheelDiameter           = builder.wheelDiameter;
        this.maxDriveSpeed           = builder.maxDriveSpeed;
        this.driveGearRatio          = builder.driveGearRatio;
        this.slipCurrent             = builder.slipCurrent;
        this.invertLeftSide          = builder.invertLeftSide;
        this.invertRightSide         = builder.invertRightSide;
        this.driveInertia            = builder.driveInertia;
        this.robotMass               = builder.robotMass;
        this.drivetrainType          = builder.drivetrainType;
        this.gyro                    = builder.gyro;
        this.driveMotorControlConfig = builder.driveMotorControlConfig;
        this.canbus                  = builder.canbus;
        this.wheelBase               = builder.wheelBase;
        this.bumperFrameWidth        = builder.bumperFrameWidth;
        this.bumperFrameLength       = builder.bumperFrameLength;
    }

    /**
     * Returns the track width of the drivetrain, which is the distance between the center of the
     * left and right wheels.
     *
     * @return the track width of the drivetrain, in meters.
     */
    public Distance getTrackWidth()
    {
        return trackWidth;
    }

    /**
     * Returns the wheel base of the drivetrain.
     *
     * @return The wheel base of the drivetrain, in meters.
     */
    public Distance getWheelBase()
    {
        return wheelBase;
    }

    /**
     * Returns the frame width of the drivetrain including the bumpers.
     *
     * @return The frame width of the drivetrain including the bumpers
     */
    public Distance getBumperFrameWidth()
    {
        return bumperFrameWidth;
    }

    /**
     * Returns the frame length of the drivetrain including the bumpers.
     *
     * @return The frame length of the drivetrain including the bumpers
     */
    public Distance getBumperFrameLength()
    {
        return bumperFrameLength;
    }

    /**
     * Returns the diameter of the wheels on the drivetrain.
     *
     * @return The diameter of the wheels on the drivetrain, in meters.
     */
    public Distance getWheelDiameter()
    {
        return wheelDiameter;
    }

    /**
     * Returns the maximum speed the drivetrain can travel at.
     *
     * @return the maximum speed the drivetrain can travel at, in meters per second
     */
    public LinearVelocity getMaxDriveSpeed()
    {
        return maxDriveSpeed;
    }

    /**
     * Gets the gear ratio of the drive motor.
     *
     * @return the gear ratio of the drive motor
     */
    public double getDriveGearRatio()
    {
        return driveGearRatio;
    }

    /**
     * Gets the current at which the wheels slip.
     *
     * @return the current at which the wheels slip, in Amps
     */
    public Current getSlipCurrent()
    {
        return slipCurrent;
    }

    /**
     * Returns whether the left side of the drivetrain should be inverted.
     *
     * @return true if the left side should be inverted, false otherwise
     */
    public boolean isInvertLeftSide()
    {
        return invertLeftSide;
    }

    /**
     * Gets whether the right side of the drivetrain should be inverted.
     *
     * @return true if the right side should be inverted, false otherwise
     */
    public boolean isInvertRightSide()
    {
        return invertRightSide;
    }

    /**
     * Gets the moment of inertia of the drivetrain.
     *
     * @return the moment of inertia of the drivetrain in kg*m^2
     */
    public MomentOfInertia getDriveInertia()
    {
        return driveInertia;
    }

    /**
     * Gets the robot's mass.
     *
     * @return the robot's mass in kg
     */
    public Mass getRobotMass()
    {
        return robotMass;
    }

    /**
     * Returns the type of the drivetrain.
     *
     * @return The type of the drivetrain
     */
    public String getDrivetrainType()
    {
        return drivetrainType;
    }

    /**
     * Returns the configuration for the drive motor controller.
     *
     * @return the configuration for the drive motor controller
     */
    public SmartMotorControllerConfig getDriveMotorControlConfig()
    {
        return driveMotorControlConfig;
    }

    /**
     * Gets the gyro associated with this drivetrain configuration.
     *
     * @return the gyro associated with this drivetrain configuration
     */
    public GenericGyro getGyro()
    {
        return gyro;
    }

    /**
     * Gets the canbus string associated with this drivetrain configuration.
     *
     * @return the canbus string associated with this drivetrain configuration
     */
    public String getCanBusName()
    {
        return canbus;
    }

    // ***********************************************

    /** A builder for drivetrain configurations */
    public static class DrivetrainConfigBuilder
    {
        protected Distance trackWidth;
        protected Distance wheelBase;
        protected Distance wheelDiameter;
        protected Distance bumperFrameWidth;
        protected Distance bumperFrameLength;
        protected LinearVelocity maxDriveSpeed;
        protected double driveGearRatio;
        protected Current slipCurrent;
        protected boolean invertLeftSide;
        protected boolean invertRightSide;
        protected MomentOfInertia driveInertia;
        protected Mass robotMass;
        protected String drivetrainType;
        protected GenericGyro gyro;
        protected SmartMotorControllerConfig driveMotorControlConfig;
        protected String canbus;

        /** Creates a new drivetrain configuration builder. */
        protected DrivetrainConfigBuilder() {}

        /**
         * Builds a new drivetrain configuration from the values set in this builder.
         *
         * @return a new drivetrain configuration
         */
        public DrivetrainConfig build(DrivetrainConstantsJson constants, SubsystemBase subsystem)
        {
            withDriveGearRatio(new MechanismGearing(GearBox.fromStages(constants.driveGearRatio))
                                   .getRotorToMechanismRatio());
            withInvertLeftSide(constants.invertLeftSide);
            withInvertRightSide(constants.invertRightSide);
            withDriveInertia(UnitsParser.parseMomentOfInertia(constants.driveInertia));
            withMaxDriveSpeed(UnitsParser.parseVelocity(constants.maxDriveSpeed));
            withRobotMass(UnitsParser.parseMass(constants.robotMass));
            withTrackWidth(UnitsParser.parseDistance(constants.trackWidth));
            withWheelBase(UnitsParser.parseDistance(constants.wheelBase));
            withBumperFrameLength(UnitsParser.parseDistance(constants.bumperFrameLength));
            withBumperFrameWidth(UnitsParser.parseDistance(constants.bumperFrameWidth));
            withWheelDiameter(UnitsParser.parseDistance(constants.wheelDiameter));
            withSlipCurrent(UnitsParser.parseAmps(constants.slipCurrent));
            withGyro((GenericGyro) constants.gyro.configure(subsystem));
            withCanbus(constants.canbus);

            return new DrivetrainConfig(this);
        }

        /**
         * Set the track width of the drivetrain.
         *
         * <p>This value should be set to the distance between the center of the left and right
         * wheels on the drivetrain in meters. This value is used by the drivetrain to calculate its
         * linear velocity.
         *
         * @param trackWidth the track width of the drivetrain in meters
         * @return this builder
         */
        public DrivetrainConfigBuilder withTrackWidth(Distance trackWidth)
        {
            this.trackWidth = trackWidth;
            return this;
        }

        /**
         * Set the wheel base of the drivetrain.
         *
         * <p>This value should be set to the distance between the centers of the front and back
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

        /**
         * Set the diameter of the wheels on the drivetrain.
         *
         * <p>This value should be set to the diameter of the wheels on the drivetrain in meters.
         * This value is used by the drivetrain to calculate its linear velocity.
         *
         * @param wheelDiameter the diameter of the wheels on the drivetrain in meters
         * @return this builder
         */
        public DrivetrainConfigBuilder withWheelDiameter(Distance wheelDiameter)
        {
            this.wheelDiameter = wheelDiameter;
            return this;
        }

        /**
         * Set the maximum speed of the drivetrain.
         *
         * <p>This value should be set to the maximum speed that the drivetrain can reach. This
         * value is used to limit the speed of the drivetrain and prevent it from running too
         * quickly.
         *
         * @param maxDriveSpeed the maximum speed of the drivetrain
         * @return this builder
         */
        public DrivetrainConfigBuilder withMaxDriveSpeed(LinearVelocity maxDriveSpeed)
        {
            this.maxDriveSpeed = maxDriveSpeed;
            return this;
        }

        /**
         * Set the gear ratio of the drivetrain.
         *
         * <p>The gear ratio is the ratio of the motor's rotational speed to the wheel's linear
         * speed. This value should be set to the gear ratio of the drivetrain.
         *
         * @param driveGearRatio the gear ratio of the drivetrain
         * @return this builder
         */
        public DrivetrainConfigBuilder withDriveGearRatio(double driveGearRatio)
        {
            this.driveGearRatio = driveGearRatio;
            return this;
        }

        /**
         * Set the current at which the robot is slipping.
         *
         * <p>The slip current is the current at which the robot's wheels are slipping. This value
         * should be set to the current at which the robot's wheels start to slip.
         *
         * @param slipCurrent the current at which the robot's wheels are slipping
         * @return this builder
         */
        public DrivetrainConfigBuilder withSlipCurrent(Current slipCurrent)
        {
            this.slipCurrent = slipCurrent;
            return this;
        }

        /**
         * Set whether the left side of the drivetrain should be inverted.
         *
         * <p>Inverting a side of the drivetrain means that the motor will turn in the opposite
         * direction when given a positive output. This can be useful for drivetrains that have
         * their motors mounted in a way that causes them to naturally turn in the opposite
         * direction.
         *
         * @param invertLeftSide whether the left side should be inverted
         * @return this builder
         */
        public DrivetrainConfigBuilder withInvertLeftSide(boolean invertLeftSide)
        {
            this.invertLeftSide = invertLeftSide;
            return this;
        }

        /**
         * Set whether the right side of the drivetrain should be inverted.
         *
         * @param invertRightSide whether the right side should be inverted
         * @return this builder
         */
        public DrivetrainConfigBuilder withInvertRightSide(boolean invertRightSide)
        {
            this.invertRightSide = invertRightSide;
            return this;
        }

        /**
         * Set the moment of inertia of the drivetrain.
         *
         * @param driveInertia the moment of inertia of the drivetrain in kg*m^2
         * @return this builder
         */
        public DrivetrainConfigBuilder withDriveInertia(MomentOfInertia driveInertia)
        {
            this.driveInertia = driveInertia;
            return this;
        }

        /**
         * Set the robot's mass.
         *
         * @param robotMass the mass of the robot in kg
         * @return this builder
         */
        public DrivetrainConfigBuilder withRobotMass(Mass robotMass)
        {
            this.robotMass = robotMass;
            return this;
        }

        /**
         * Set the type of the drivetrain.
         *
         * @param drivetrainType the type of the drivetrain
         * @return this builder
         */
        public DrivetrainConfigBuilder withDrivetrainType(String drivetrainType)
        {
            this.drivetrainType = drivetrainType;
            return this;
        }

        /**
         * Set the gyro used by the drivetrain.
         *
         * @param gyro the gyro used by the drivetrain
         * @return this builder
         */
        public DrivetrainConfigBuilder withGyro(GenericGyro gyro)
        {
            this.gyro = gyro;
            return this;
        }

        /**
         * Set the drive motor controller configuration.
         *
         * @param driveMotorControlConfig the drive motor controller configuration
         * @return this builder
         */
        public DrivetrainConfigBuilder withDriveMotorControlConfig(
            SmartMotorControllerConfig driveMotorControlConfig)
        {
            this.driveMotorControlConfig = driveMotorControlConfig;
            return this;
        }

        /**
         * Set the canbus string for the drivetrain.
         *
         * @param canbus the canbus string for the drivetrain
         * @return this builder
         */
        public DrivetrainConfigBuilder withCanbus(String canbus)
        {
            this.canbus = canbus;
            return this;
        }

        /**
         * Set the bumper frame width of the drivetrain.
         *
         * <p>This value should be set to the width of the bumper frame in meters. This value is
         * used by the drivetrain to calculate its linear velocity.
         *
         * @param bumperFrameWidth the bumper frame width of the drivetrain in meters
         * @return this builder
         */
        public DrivetrainConfigBuilder withBumperFrameWidth(Distance bumperFrameWidth)
        {
            this.bumperFrameWidth = bumperFrameWidth;
            return this;
        }

        /**
         * Set the bumper frame length of the drivetrain.
         *
         * <p>This value should be set to the length of the bumper frame in meters. This value is
         * used by the drivetrain to calculate its linear velocity.
         *
         * @param bumperFrameLength the bumper frame length of the drivetrain in meters
         * @return this builder
         */
        public DrivetrainConfigBuilder withBumperFrameLength(Distance bumperFrameLength)
        {
            this.bumperFrameLength = bumperFrameLength;
            return this;
        }
    }
}
