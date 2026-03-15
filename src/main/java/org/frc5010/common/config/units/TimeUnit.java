package org.frc5010.common.config.units;

/**
 * Enum representing time unit types. Each enum value can be parsed from multiple string
 * representations.
 */
public enum TimeUnit {
    SECONDS("s", "sec", "second", "seconds"),
    MILLISECONDS("ms", "millisecond", "milliseconds"),
    MICROSECONDS("us", "microsecond", "microseconds"),
    NANOSECONDS("ns", "nanosecond", "nanoseconds"),
    MINUTES("min", "minute", "minutes"),
    HOURS("h", "hour", "hours"),
    DAYS("d", "day", "days");

    private final String[] aliases;

    TimeUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into a TimeUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching TimeUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static TimeUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (TimeUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown time unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
