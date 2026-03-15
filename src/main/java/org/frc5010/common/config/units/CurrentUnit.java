package org.frc5010.common.config.units;

/**
 * Enum representing current unit types. Each enum value can be parsed from multiple string
 * representations.
 */
public enum CurrentUnit {
    AMPS("a", "amp", "amps", "ampere", "amperes"),
    MILLIAMPS("ma", "milliamp", "milliamps", "milliampere", "milliamperes"),
    MICROAMPS("ua", "microamp", "microamps", "microampere", "microamperes");

    private final String[] aliases;

    CurrentUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into a CurrentUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching CurrentUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static CurrentUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (CurrentUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown current unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
