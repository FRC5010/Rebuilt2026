package org.frc5010.common.config.units;

/**
 * Enum representing angle unit types. Each enum value can be parsed from multiple string
 * representations.
 */
public enum AngleUnit {
    DEGREES("deg", "degrees"),
    RADIANS("rad", "radians"),
    ROTATIONS("rot", "rotation", "rotations");

    private final String[] aliases;

    AngleUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into an AngleUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching AngleUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static AngleUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (AngleUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown angle unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
