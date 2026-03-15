package org.frc5010.common.config.units;

/**
 * Enum representing distance unit types. Each enum value can be parsed from multiple string
 * representations.
 */
public enum DistanceUnit {
    METERS("m", "meter", "meters"),
    INCHES("in", "inch", "inches"),
    FEET("ft", "foot", "feet"),
    MILLIMETERS("mm", "millimeter", "millimeters"),
    CENTIMETERS("cm", "centimeter", "centimeters"),
    YARDS("yd", "yard", "yards");

    private final String[] aliases;

    DistanceUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into a DistanceUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching DistanceUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static DistanceUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (DistanceUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown distance unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
