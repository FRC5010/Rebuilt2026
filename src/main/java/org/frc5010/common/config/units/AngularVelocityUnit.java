package org.frc5010.common.config.units;

/**
 * Enum representing angular velocity unit types. Each enum value can be parsed from multiple string
 * representations.
 */
public enum AngularVelocityUnit {
    DEGREES_PER_SECOND("deg/s",
                       "deg/sec",
                       "deg/second",
                       "degrees/s",
                       "degrees/sec",
                       "degrees/second"),
    RADIANS_PER_SECOND("rad/s",
                       "rad/sec",
                       "rad/second",
                       "rads/s",
                       "rads/sec",
                       "rads/second",
                       "radians",
                       "radians/s",
                       "radians/sec",
                       "radians/second"),
    REVOLUTIONS_PER_MINUTE("rpm", "RPM"),
    ROTATIONS_PER_SECOND("rps", "RPS");

    private final String[] aliases;

    AngularVelocityUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into an AngularVelocityUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching AngularVelocityUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static AngularVelocityUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (AngularVelocityUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown angular velocity unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
