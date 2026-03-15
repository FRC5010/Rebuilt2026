package org.frc5010.common.telemetry;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumSet;
import java.util.function.BooleanSupplier;
import org.frc5010.common.arch.GenericRobot.LogLevel;

/** Add a boolean to the dashboard */
public class DisplayBoolean extends DisplayableValue
{
    // Variables
    /** The value being displayed */
    protected boolean value_;
    /** The topic */
    protected BooleanTopic topic_;
    /** The publisher */
    protected BooleanPublisher publisher_;
    /** The subscriber */
    protected BooleanSubscriber subscriber_;

    // Constructor
    /**
     * Create a new display boolean
     *
     * @param defaultValue the default value
     * @param name the name of the variable being stored
     * @param table the table being stored in
     */
    public DisplayBoolean(final boolean defaultValue, final String name, final String table)
    {
        this(defaultValue, name, table, LogLevel.COMPETITION);
    }

    /**
     * Create a new display boolean
     *
     * @param defaultValue the default value
     * @param name the name of the variable being stored
     * @param table the table being stored in
     */
    public DisplayBoolean(final boolean defaultValue,
                          final String name,
                          final String table,
                          final LogLevel logLevel)
    {
        super(name, table, logLevel);
        value_ = defaultValue;
        if (isDisplayed_)
        {
            topic_     = NetworkTableInstance.getDefault().getTable(table_).getBooleanTopic(name_);
            publisher_ = topic_.publish();
            publisher_.setDefault(value_);
        }
        if (LogLevel.CONFIG == logLevel)
        {
            if (isDisplayed_)
                topic_.setPersistent(true);
            if (DisplayValuesHelper.isAtLogLevel(LogLevel.CONFIG))
            {
                subscriber_     = topic_.subscribe(value_);
                listenerHandle_ = NetworkTableInstance.getDefault().addListener(
                    subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    event -> { setValue(event.valueData.value.getBoolean(), false); });
                setValue(subscriber_.get(), false);
            }
        }
    }

    // Getters
    /**
     * Get the value
     *
     * @return the value
     */
    public synchronized boolean getValue()
    {
        return value_;
    }

    // Setters
    /**
     * Set the value
     *
     * @param value the value to set
     */
    public synchronized void setValue(final boolean value)
    {
        setValue(value, true);
    }

    /**
     * Set the value
     *
     * @param value the value to set
     * @param publish whether or not to publish the value
     */
    public synchronized void setValue(final boolean value, final boolean publish)
    {
        value_ = value;
        if (publish && isDisplayed_)
        {
            publisher_.set(value_);
        }
    }

    /**
     * Registers a listener to the display values helper to update this boolean with the result of
     * the given supplier when the listener is called.
     *
     * @param supplier - supplier of the boolean to update this with
     * @return this object
     */
    public DisplayBoolean updateWith(BooleanSupplier supplier)
    {
        displayValuesHelper_.registerListener(() -> setValue(supplier.getAsBoolean()));
        return this;
    }
}
