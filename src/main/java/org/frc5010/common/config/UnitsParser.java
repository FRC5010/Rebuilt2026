// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Ounces;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.config.units.AngleUnit;
import org.frc5010.common.config.units.AngularAccelerationUnit;
import org.frc5010.common.config.units.AngularVelocityUnit;
import org.frc5010.common.config.units.CurrentUnit;
import org.frc5010.common.config.units.DistanceUnit;
import org.frc5010.common.config.units.LinearAccelerationUnit;
import org.frc5010.common.config.units.LinearVelocityUnit;
import org.frc5010.common.config.units.MassUnit;
import org.frc5010.common.config.units.MomentOfInertiaUnit;
import org.frc5010.common.config.units.TimeUnit;
import org.frc5010.common.config.units.VoltageUnit;

/** A class that converts a magnitude and a unit into a {@link Measurement} object. */
public class UnitsParser
{
    /**
     * Deprecated: Use the specific unit enum classes instead. These constants are provided for
     * backwards compatibility with existing code.
     *
     * @deprecated Use {@link DistanceUnit#METERS}, {@link LinearVelocityUnit#METERS_PER_SECOND},
     *     etc.
     */
    @Deprecated public static final String CM = "cm";

    @Deprecated public static final String MM     = "mm";
    @Deprecated public static final String IN     = "in";
    @Deprecated public static final String FT     = "ft";
    @Deprecated public static final String YD     = "yd";
    @Deprecated public static final String MPS    = "m/s";
    @Deprecated public static final String CMPS   = "cm/s";
    @Deprecated public static final String MPS2   = "m/s^2";
    @Deprecated public static final String CMPS2  = "cm/s^2";
    @Deprecated public static final String DEG    = "deg";
    @Deprecated public static final String RAD    = "rad";
    @Deprecated public static final String DEGPS  = "deg/s";
    @Deprecated public static final String RADPS  = "rad/s";
    @Deprecated public static final String rpm    = "rpm";
    @Deprecated public static final String rps    = "rps";
    @Deprecated public static final String DEGPS2 = "deg/s^2";
    @Deprecated public static final String RADPS2 = "rad/s^2";
    @Deprecated public static final String RPMPS  = "rpmps";
    @Deprecated public static final String RPS2   = "rps^2";
    @Deprecated public static final String AMPS   = "amps";
    @Deprecated public static final String VOLTS  = "volts";
    @Deprecated public static final String SEC    = "sec";
    @Deprecated public static final String MS     = "ms";
    @Deprecated public static final String US     = "us";
    @Deprecated public static final String NS     = "ns";
    @Deprecated public static final String KG     = "kg";
    @Deprecated public static final String G      = "g";
    @Deprecated public static final String MG     = "mg";
    @Deprecated public static final String OZ     = "oz";
    @Deprecated public static final String LBS    = "lbs";
    @Deprecated public static final String STONE  = "stone";
    @Deprecated public static final String TONS   = "tons";
    @Deprecated private static final String FPS   = "ft/s";
    @Deprecated private static final String FPS2  = "ft/s^2";
    /**
     * Converts a magnitude and a unit into a {@link Distance} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the distance.
     * @return The {@link Distance} object.
     */
    public static Distance parseDistance(UnitValueJson unitValueJson)
    {
        return parseDistance(unitValueJson.val, unitValueJson.uom);
    }

    /**
     * Converts a magnitude and a unit into a {@link Distance} object.
     *
     * @param magnitude The magnitude of the distance.
     * @param unit The unit of the distance. Can be "meters", "feet", "inches", "mm", "cm", or "yd".
     *     (case insensitive)
     * @return The {@link Distance} object.
     */
    public static Distance parseDistance(double magnitude, String unit)
    {
        DistanceUnit distanceUnit = DistanceUnit.fromString(unit);
        switch (distanceUnit)
        {
            case METERS:
                return Meters.of(magnitude);
            case INCHES:
                return Inches.of(magnitude);
            case FEET:
                return Feet.of(magnitude);
            case MILLIMETERS:
                return Millimeters.of(magnitude);
            case CENTIMETERS:
                return Centimeters.of(magnitude);
            case YARDS:
                return Feet.of(magnitude * 3);
            default:
                System.err.println("Unexpected distance unit: " + distanceUnit);
                return Meters.of(magnitude);
        }
    }

    /**
     * Converts a magnitude and a unit into a {@link LinearVelocity} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the velocity.
     * @return The {@link LinearVelocity} object.
     */
    public static LinearVelocity parseVelocity(UnitValueJson unitValueJson)
    {
        return parseVelocity(unitValueJson.val, unitValueJson.uom);
    }

    /**
     * Converts a magnitude and a unit into a {@link LinearVelocity} object.
     *
     * @param magnitude The magnitude of the velocity.
     * @param unit The unit of the velocity. Can be "meters/second", "inches/second", "feet/second",
     *     "millimeters/second", or "centimeters/second". (case insensitive)
     * @return The {@link LinearVelocity} object.
     */
    public static LinearVelocity parseVelocity(double magnitude, String unit)
    {
        LinearVelocityUnit velocityUnit = LinearVelocityUnit.fromString(unit);
        switch (velocityUnit)
        {
            case METERS_PER_SECOND:
                return MetersPerSecond.of(magnitude);
            case INCHES_PER_SECOND:
                return InchesPerSecond.of(magnitude);
            case FEET_PER_SECOND:
                return FeetPerSecond.of(magnitude);
            case MILLIMETERS_PER_SECOND:
                return MetersPerSecond.of(magnitude / 1000);
            case CENTIMETERS_PER_SECOND:
                return MetersPerSecond.of(magnitude / 100);
            default:
                System.err.println("Unexpected linear velocity unit: " + velocityUnit);
                break;
        }
        return MetersPerSecond.of(magnitude);
    }

    /**
     * Converts a {@link UnitValueJson} object into a {@link LinearAcceleration} object.
     *
     * @param unitValueJson The {@link UnitValueJson} object containing the magnitude and unit of
     *     the acceleration.
     * @return The {@link LinearAcceleration} object.
     */
    public static LinearAcceleration parseAccelleration(UnitValueJson unitValueJson)
    {
        return parseAccelleration(unitValueJson.val, unitValueJson.uom);
    }

    /**
     * Converts a magnitude and a unit into a {@link LinearAcceleration} object.
     *
     * @param magnitude The magnitude of the acceleration.
     * @param unit The unit of the acceleration. Can be "m/s/s", "m/s2", "m/s^2", "m/sec/sec",
     *     "m/sec2", "m/sec^2", "meters/sec/sec", "meters/sec2", "meters/sec^2",
     *     "meters/second/second", "meters/second2", "meters/second^2", "in/s/s", "in/s2", "in/s^2",
     *     "in/sec/sec", "in/sec2", "in/sec^2", "inches/sec/sec", "inches/sec2", "inches/sec^2",
     *     "inches/second/second", "inches/second2", "inches/second^2", "ft/s/s", "ft/s2", "ft/s^2",
     *     "ft/sec/sec", "ft/sec2", "ft/sec^2", "feet/sec/sec", "feet/sec2", "feet/sec^2",
     *     "feet/second/second", "feet/second2", "feet/second^2", "mm/s/s", "mm/s2", "mm/s^2",
     *     "millimeter/sec/sec", "millimeter/sec2", "millimeter/sec^2", "millimeters/sec/sec",
     *     "millimeters/sec2", "millimeters/sec^2", "millimeters/second/second",
     *     "millimeters/second2", "millimeters/second^2", "cm/s^2", "cm/sec^2", "cm/second^2",
     *     "centimeter/sec^2", "centimeter/second^2", "centimeters/sec^2", "centimeters/second^2",
     *     "cms/s^2", "cms/sec^2", "cms/second^2", "cm/s/s", "cm/sec/sec", "cm/second/second",
     *     "centimeters/sec/sec", "centimeters/second/second". (case insensitive)
     * @return The {@link LinearAcceleration} object.
     */
    public static LinearAcceleration parseAccelleration(double magnitude, String unit)
    {
        LinearAccelerationUnit accelUnit = LinearAccelerationUnit.fromString(unit);
        switch (accelUnit)
        {
            case METERS_PER_SECOND_SQUARED:
                return MetersPerSecondPerSecond.of(magnitude);
            case INCHES_PER_SECOND_SQUARED:
                return InchesPerSecond.of(magnitude).per(Second);
            case FEET_PER_SECOND_SQUARED:
                return FeetPerSecondPerSecond.of(magnitude);
            case MILLIMETERS_PER_SECOND_SQUARED:
                return MetersPerSecondPerSecond.of(magnitude / 1000);
            case CENTIMETERS_PER_SECOND_SQUARED:
                return MetersPerSecondPerSecond.of(magnitude / 100);
            default:
                System.err.println("Unexpected linear acceleration unit: " + accelUnit);
                break;
        }
        return MetersPerSecondPerSecond.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Current} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the current.
     * @return The {@link Current} object.
     */
    public static Current parseAmps(UnitValueJson unitValueJson)
    {
        return parseAmps(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Current} object.
     *
     * @param magnitude The magnitude of the current.
     * @param unit The unit of the current. Can be "a", "amp", "amps", "ampere", "amperes", "ma",
     *     "milliamp", "milliamps", "milliampere", "milliamperes", "ua", "microamp", "microamps",
     *     "microampere", or "microamperes". (case insensitive)
     * @return The {@link Current} object.
     */
    public static Current parseAmps(double magnitude, String unit)
    {
        CurrentUnit currentUnit = CurrentUnit.fromString(unit);
        switch (currentUnit)
        {
            case AMPS:
                return Amps.of(magnitude);
            case MILLIAMPS:
                return Amps.of(magnitude * 0.001);
            case MICROAMPS:
                return Amps.of(magnitude * 0.000001);
            default:
                System.err.println("Unexpected current unit: " + currentUnit);
                break;
        }
        return Amps.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Voltage} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the voltage.
     * @return The {@link Voltage} object.
     */
    public static Voltage parseVolts(UnitValueJson unitValueJson)
    {
        return parseVolts(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Voltage} object.
     *
     * @param magnitude The magnitude of the voltage.
     * @param unit The unit of the voltage. Can be "v", "volt", "volts", "voltage", "mv",
     *     "millivolt", "millivolts", "uv", "microvolt", "microvolts", "kv", "kilovolt", or
     *     "kilovolts". (case insensitive)
     * @return The {@link Voltage} object.
     */
    public static Voltage parseVolts(double magnitude, String unit)
    {
        VoltageUnit voltageUnit = VoltageUnit.fromString(unit);
        switch (voltageUnit)
        {
            case VOLTS:
                return Volts.of(magnitude);
            case MILLIVOLTS:
                return Volts.of(magnitude * 0.001);
            case MICROVOLTS:
                return Volts.of(magnitude * 0.000001);
            case KILOVOLTS:
                return Volts.of(magnitude * 1000);
            default:
                System.err.println("Unexpected voltage unit: " + voltageUnit);
                break;
        }
        return Volts.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Time} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the time.
     * @return The {@link Time} object.
     */
    public static Time parseTime(UnitValueJson unitValueJson)
    {
        return parseTime(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Time} object.
     *
     * @param magnitude The magnitude of the time.
     * @param unit The unit of the time. Can be "s", "sec", "second", "seconds", "ms",
     *     "millisecond", "milliseconds", "us", "microsecond", "microseconds", "ns", "nanosecond",
     *     "nanoseconds", "min", "minute", "minutes", "h", "hour", "hours", "d", "day", "days".
     *     (case insensitive)
     * @return The {@link Time} object.
     */
    public static Time parseTime(double magnitude, String unit)
    {
        TimeUnit timeUnit = TimeUnit.fromString(unit);
        switch (timeUnit)
        {
            case SECONDS:
                return Seconds.of(magnitude);
            case MILLISECONDS:
                return Seconds.of(magnitude * 0.001);
            case MICROSECONDS:
                return Seconds.of(magnitude * 0.000001);
            case NANOSECONDS:
                return Seconds.of(magnitude * 0.000000001);
            case MINUTES:
                return Seconds.of(magnitude * 60);
            case HOURS:
                return Seconds.of(magnitude * 3600);
            case DAYS:
                return Seconds.of(magnitude * 86400);
            default:
                System.err.println("Unexpected time unit: " + timeUnit);
                break;
        }
        return Seconds.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Mass} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the weight.
     * @return The {@link Mass} object.
     */
    public static Mass parseMass(UnitValueJson unitValueJson)
    {
        return parseMass(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Mass} object.
     *
     * @param magnitude The magnitude of the weight.
     * @param unit The unit of the weight. Can be "kg", "g", "mg", "t", "oz", "lb", "st", "pound",
     *     "pounds", "kilograms", "grams", "milligrams", "ton", "tons", "stone", or "stones". (case
     *     insensitive)
     * @return The {@link Mass} object.
     */
    public static Mass parseMass(double magnitude, String unit)
    {
        MassUnit massUnit = MassUnit.fromString(unit);
        switch (massUnit)
        {
            case KILOGRAMS:
                return Kilograms.of(magnitude);
            case GRAMS:
                return Kilograms.of(magnitude * 0.001);
            case MILLIGRAMS:
                return Kilograms.of(magnitude * 0.000001);
            case TONS:
                return Kilograms.of(magnitude * 1000);
            case OUNCES:
                return Ounces.of(magnitude);
            case POUNDS:
                return Pounds.of(magnitude);
            case STONE:
                return Kilograms.of(magnitude * 6.35029);
            default:
                System.err.println("Unexpected mass unit: " + massUnit);
                break;
        }
        return Kilograms.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Angle} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the angle.
     * @return The {@link Angle} object.
     */
    public static Angle parseAngle(UnitValueJson unitValueJson)
    {
        return parseAngle(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Angle} object.
     *
     * @param magnitude The magnitude of the angle.
     * @param unit The unit of the angle. Can be "deg", "degrees", "rad", or "radians". (case
     *     insensitive)
     * @return The {@link Angle} object.
     */
    public static Angle parseAngle(double magnitude, String unit)
    {
        AngleUnit angleUnit = AngleUnit.fromString(unit);
        switch (angleUnit)
        {
            case DEGREES:
                return Degrees.of(magnitude);
            case RADIANS:
                return Radians.of(magnitude);
            case ROTATIONS:
                return Rotations.of(magnitude);
            default:
                System.err.println("Unexpected angle unit: " + angleUnit);
                return Degrees.of(magnitude);
        }
    }

    /**
     * Converts a magnitude and a unit into an {@link AngularAcceleration} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the angular
     *     acceleration.
     * @return The {@link AngularAcceleration} object.
     */
    public static AngularAcceleration parseAngularAcceleration(UnitValueJson unitValueJson)
    {
        return parseAngularAcceleration(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into an {@link AngularVelocity} object.
     *
     * @param magnitude The magnitude of the angular velocity.
     * @param unit The unit of the angular velocity. Can be "deg/s", "deg/sec", "deg/second",
     *     "degrees/s", "degrees/sec", "degrees/second", "rad/s", "rad/sec", "rad/second", "rads/s",
     *     "rads/sec", "rads/second", "radians", "radians/s", "radians/sec", "radians/second". (case
     *     insensitive)
     * @return The {@link AngularVelocity} object.
     */
    public static AngularVelocity parseAngularVelocity(double magnitude, String unit)
    {
        AngularVelocityUnit angVelUnit = AngularVelocityUnit.fromString(unit);
        switch (angVelUnit)
        {
            case DEGREES_PER_SECOND:
                return DegreesPerSecond.of(magnitude);
            case REVOLUTIONS_PER_MINUTE:
                return RPM.of(magnitude);
            case ROTATIONS_PER_SECOND:
                return RotationsPerSecond.of(magnitude);
            case RADIANS_PER_SECOND:
                return RadiansPerSecond.of(magnitude);
            default:
                System.err.println("Unexpected angular velocity unit: " + angVelUnit);
                return DegreesPerSecond.of(magnitude);
        }
    }

    /**
     * Converts a magnitude and a unit into an {@link AngularVelocity} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the angular velocity.
     * @return The {@link AngularVelocity} object.
     */
    public static AngularVelocity parseAngularVelocity(UnitValueJson unitValueJson)
    {
        return parseAngularVelocity(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link AngularAcceleration} object.
     *
     * @param magnitude The magnitude of the angular acceleration.
     * @param unit The unit of the angular acceleration. Can be "deg/s/s", "deg/s2", "deg/s^2",
     *     "degrees/s/s", "degrees/s2", "degrees/s^2", "rads/s/s", "rads/s2", "rads/s^2", "radians",
     *     "radians/s/s", "radians/s2", "radians/s^2". (case insensitive)
     * @return The {@link AngularAcceleration} object.
     */
    public static AngularAcceleration parseAngularAcceleration(double magnitude, String unit)
    {
        AngularAccelerationUnit angAccelUnit = AngularAccelerationUnit.fromString(unit);
        switch (angAccelUnit)
        {
            case DEGREES_PER_SECOND_SQUARED:
                return DegreesPerSecondPerSecond.of(magnitude);
            case REVOLUTIONS_PER_MINUTE_PER_SECOND:
                return RPM.of(magnitude).per(Second);
            case ROTATIONS_PER_SECOND_SQUARED:
                return RotationsPerSecondPerSecond.of(magnitude);
            case RADIANS_PER_SECOND_SQUARED:
                return RadiansPerSecondPerSecond.of(magnitude);
            default:
                System.err.println("Unexpected angular acceleration unit: " + angAccelUnit);
                return DegreesPerSecondPerSecond.of(magnitude);
        }
    }

    /**
     * Converts a magnitude and a unit into a {@link MomentOfInertia} object.
     *
     * @param unitValueJson A json object containing the magnitude and unit of the moment of
     *     inertia.
     * @return The {@link MomentOfInertia} object.
     */
    public static MomentOfInertia parseMomentOfInertia(UnitValueJson unitValueJson)
    {
        return parseMomentOfInertia(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link MomentOfInertia} object.
     *
     * @param magnitude The magnitude of the moment of inertia
     * @param unit The unit of the moment moment @return The {@link MomentOfInertia} object
     */
    public static MomentOfInertia parseMomentOfInertia(double magnitude, String unit)
    {
        MomentOfInertiaUnit moiUnit = MomentOfInertiaUnit.fromString(unit);
        switch (moiUnit)
        {
            case KILOGRAM_SQUARE_METERS:
                return KilogramSquareMeters.of(magnitude);
            default:
                System.err.println("Unexpected moment of inertia unit: " + moiUnit);
                return KilogramSquareMeters.of(magnitude);
        }
    }
}
