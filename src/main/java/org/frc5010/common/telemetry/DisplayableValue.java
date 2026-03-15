package org.frc5010.common.telemetry;

import org.frc5010.common.arch.GenericRobot.LogLevel;

public abstract class DisplayableValue
{
    /** The name */
    protected final String name_;
    /** The table */
    protected final String table_;
    /** The listener handle */
    protected int listenerHandle_;
    /** Display mode */
    protected final boolean isDisplayed_;
    /** Display values helper */
    protected DisplayValuesHelper displayValuesHelper_;

    /**
     * Add a value to the dashboard
     *
     * @param name the name of the variable
     * @param table the name of the table
     */
    public DisplayableValue(final String name, final String table, LogLevel logLevel)
    {
        name_        = name;
        table_       = table;
        isDisplayed_ = DisplayValuesHelper.isAtLogLevel(logLevel);
    }

    public DisplayableValue setDisplayValuesHelper(DisplayValuesHelper displayValuesHelper)
    {
        displayValuesHelper_ = displayValuesHelper;
        return this;
    }
}
