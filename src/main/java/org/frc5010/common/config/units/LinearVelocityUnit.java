package org.frc5010.common.config.units;

/**
 * Enum representing linear velocity unit types. Each enum value can be parsed from multiple string
 * representations.
 */
public enum LinearVelocityUnit {
    METERS_PER_SECOND("m/s",
                      "m/sec",
                      "mps",
                      "meter/sec",
                      "meters/sec",
                      "meter/second",
                      "meters/second"),
    INCHES_PER_SECOND("in/s", "in/sec", "inch/sec", "inches/sec", "inch/second", "inches/second"),
    FEET_PER_SECOND("ft/s", "ft/sec", "foot/sec", "feet/sec", "foot/second", "feet/second"),
    MILLIMETERS_PER_SECOND("mm/s",
                           "mm/sec",
                           "millimeter/sec",
                           "millimeters/sec",
                           "millimeter/second",
                           "millimeters/second"),
    CENTIMETERS_PER_SECOND("cm/s",
                           "cm/sec",
                           "centimeter/sec",
                           "centimeters/sec",
                           "centimeter/second",
                           "centimeters/second");

    private final String[] aliases;

    LinearVelocityUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into a LinearVelocityUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching LinearVelocityUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static LinearVelocityUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (LinearVelocityUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown linear velocity unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
