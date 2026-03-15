package org.frc5010.common.config.units;

/**
 * Enum representing voltage unit types. Each enum value can be parsed from multiple string
 * representations.
 */
public enum VoltageUnit {
    VOLTS("v", "volt", "volts", "voltage"),
    MILLIVOLTS("mv", "millivolt", "millivolts"),
    MICROVOLTS("uv", "microvolt", "microvolts"),
    KILOVOLTS("kv", "kilovolt", "kilovolts");

    private final String[] aliases;

    VoltageUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into a VoltageUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching VoltageUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static VoltageUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (VoltageUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown voltage unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
