package org.frc5010.common.config.units;

/**
 * Enum representing mass unit types. Each enum value can be parsed from multiple string
 * representations.
 */
public enum MassUnit {
    KILOGRAMS("kg", "kgs", "kilogram", "kilograms"),
    GRAMS("g", "gram", "grams"),
    MILLIGRAMS("mg", "milligram", "milligrams"),
    OUNCES("oz", "ounce", "ounces"),
    POUNDS("lbs", "lb", "pound", "pounds"),
    STONE("st", "stone", "stones"),
    TONS("t", "tons", "ton");

    private final String[] aliases;

    MassUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into a MassUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching MassUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static MassUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (MassUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown mass unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
