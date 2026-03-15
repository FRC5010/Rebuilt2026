package org.frc5010.common.telemetry;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.EnumSet;
import java.util.function.LongSupplier;
import org.frc5010.common.arch.GenericRobot.LogLevel;

/** Add a long to the dashboard */
public class DisplayLong extends DisplayableValue
{
    // Variables
    /** The value */
    protected long value_;
    /** The topic */
    protected IntegerTopic topic_;
    /** The publisher */
    protected IntegerPublisher publisher_;
    /** The subscriber */
    protected IntegerSubscriber subscriber_;

    // Constructor
    /**
     * Add a long to the dashboard
     *
     * @param defaultValue the default value
     * @param name the name
     * @param table the table
     */
    public DisplayLong(final long defaultValue, final String name, final String table)
    {
        this(defaultValue, name, table, LogLevel.COMPETITION);
    }

    /**
     * Add a long to the dashboard
     *
     * @param defaultValue the default value
     * @param name the name
     * @param table the table
     * @param logLevel the log level
     */
    public DisplayLong(final long defaultValue,
                       final String name,
                       final String table,
                       final LogLevel logLevel)
    {
        super(name, table, logLevel);
        value_ = defaultValue;
        if (isDisplayed_)
        {
            topic_     = NetworkTableInstance.getDefault().getTable(table_).getIntegerTopic(name_);
            publisher_ = topic_.publish();
            if (LogLevel.CONFIG == logLevel)
            {
                if (isDisplayed_)
                    topic_.setPersistent(true);
                if (DisplayValuesHelper.isAtLogLevel(LogLevel.CONFIG))
                {
                    subscriber_     = topic_.subscribe(value_);
                    listenerHandle_ = NetworkTableInstance.getDefault().addListener(
                        subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        event -> { setValue(event.valueData.value.getInteger(), false); });
                    setValue(subscriber_.get(), false);
                }
                publisher_.setDefault(value_);
            }
        }
    }

    // Getters
    /**
     * Get the value
     *
     * @return the value
     */
    public synchronized long getValue()
    {
        return value_;
    }

    // Setters
    /**
     * Set the value
     *
     * @param value the value
     */
    public synchronized void setValue(final long value)
    {
        setValue(value, true);
    }

    /**
     * Set the value
     *
     * @param value the value
     * @param publish - whether or not to publish
     */
    public synchronized void setValue(final long value, final boolean publish)
    {
        value_ = value;
        if (publish && isDisplayed_)
        {
            publisher_.set(value_);
        }
    }

    /**
     * Registers a listener to the display values helper to update this long with the result of the
     * given supplier when the listener is called.
     *
     * @param supplier - supplier of the long to update this with
     * @return this object
     */
    public DisplayLong updateWith(LongSupplier supplier)
    {
        displayValuesHelper_.registerListener(() -> setValue(supplier.getAsLong()));
        return this;
    }
}
