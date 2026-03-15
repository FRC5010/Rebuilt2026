package org.frc5010.common.config.units;

/**
 * Enum representing moment of inertia unit types. Each enum value can be parsed from multiple
 * string representations.
 */
public enum MomentOfInertiaUnit {
    KILOGRAM_SQUARE_METERS("kg*m^2", "kg*m2", "kg*sqm");

    private final String[] aliases;

    MomentOfInertiaUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into a MomentOfInertiaUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching MomentOfInertiaUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static MomentOfInertiaUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (MomentOfInertiaUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown moment of inertia unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
