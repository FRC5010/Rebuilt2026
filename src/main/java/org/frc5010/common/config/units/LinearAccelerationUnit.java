package org.frc5010.common.config.units;

/**
 * Enum representing linear acceleration unit types. Each enum value can be parsed from multiple
 * string representations.
 */
public enum LinearAccelerationUnit {
    METERS_PER_SECOND_SQUARED("m/s^2",
                              "m/s/s",
                              "m/s2",
                              "m/sec/sec",
                              "m/sec2",
                              "m/sec^2",
                              "meters/sec/sec",
                              "meters/sec2",
                              "meters/sec^2",
                              "meters/second/second",
                              "meters/second2",
                              "meters/second^2"),
    INCHES_PER_SECOND_SQUARED("in/s^2",
                              "in/s/s",
                              "in/s2",
                              "in/sec/sec",
                              "in/sec2",
                              "in/sec^2",
                              "inches/sec/sec",
                              "inches/sec2",
                              "inches/sec^2",
                              "inches/second/second",
                              "inches/second2",
                              "inches/second^2"),
    FEET_PER_SECOND_SQUARED("ft/s^2",
                            "ft/s/s",
                            "ft/s2",
                            "ft/sec/sec",
                            "ft/sec2",
                            "ft/sec^2",
                            "feet/sec/sec",
                            "feet/sec2",
                            "feet/sec^2",
                            "feet/second/second",
                            "feet/second2",
                            "feet/second^2"),
    MILLIMETERS_PER_SECOND_SQUARED("mm/s^2",
                                   "mm/s/s",
                                   "mm/s2",
                                   "millimeter/sec/sec",
                                   "millimeter/sec2",
                                   "millimeter/sec^2",
                                   "millimeters/sec/sec",
                                   "millimeters/sec2",
                                   "millimeters/sec^2",
                                   "millimeters/second/second",
                                   "millimeters/second2",
                                   "millimeters/second^2"),
    CENTIMETERS_PER_SECOND_SQUARED("cm/s^2",
                                   "cm/sec^2",
                                   "cm/second^2",
                                   "centimeter/sec^2",
                                   "centimeter/second^2",
                                   "centimeters/sec^2",
                                   "centimeters/second^2",
                                   "cms/s^2",
                                   "cms/sec^2",
                                   "cms/second^2",
                                   "cm/s/s",
                                   "cm/sec/sec",
                                   "cm/second/second",
                                   "centimeters/sec/sec",
                                   "centimeters/second/second");

    private final String[] aliases;

    LinearAccelerationUnit(String... aliases)
    {
        this.aliases = aliases;
    }

    /**
     * Attempts to parse a string into a LinearAccelerationUnit.
     *
     * @param unitString The string representation of the unit
     * @return The matching LinearAccelerationUnit
     * @throws IllegalArgumentException if no matching unit is found
     */
    public static LinearAccelerationUnit fromString(String unitString)
    {
        if (unitString == null)
        {
            throw new IllegalArgumentException("Unit string cannot be null");
        }
        String normalized = unitString.trim().toLowerCase();
        for (LinearAccelerationUnit unit : values())
        {
            for (String alias : unit.aliases)
            {
                if (alias.equals(normalized))
                {
                    return unit;
                }
            }
        }
        throw new IllegalArgumentException("Unknown linear acceleration unit: " + unitString);
    }

    public String[] getAliases()
    {
        return aliases;
    }
}
