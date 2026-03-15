package org.frc5010.common.config.units;

/**
 * Enum representing angular acceleration unit types. Each enum value can be parsed from multiple
 * string representations.
 */
public enum AngularAccelerationUnit {
    DEGREES_PER_SECOND_SQUARED("deg/s^2",
                               "deg/s/s",
                               "deg/s2",
                               "degrees/s/s",
                               "degrees/s2",
                               "degrees/s^2"),
    RADIANS_PER_SECOND_SQUARED("rad/s^2",
                               "rads/s/s",
                               "rads/s2",
                               "rads/s^2",
                               "radians",
                               "radians/s/s",
                               "radians/s2",
                               "radians/s^2"),
    REVOLUTIONS_PER_MINUTE_PER_SECOND("rpm/s", "RPM/s", "rpm/sec", "RPM/sec"),
    ROTATIONS_PER_SECOND_SQUARED("rps^2",
                                 "RPS^2",
                                 "RPS2",
                                 "RPS/s",
                                 "RPS/sec",
                                 "rps2",
                                 "rps/s",
                                 "rps/sec");

    private final String[] aliases;

    AngularAccelerationUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into an AngularAccelerationUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching AngularAccelerationUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static AngularAccelerationUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (AngularAccelerationUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown angular acceleration unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
