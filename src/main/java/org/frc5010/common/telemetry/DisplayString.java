package org.frc5010.common.telemetry;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericRobot.LogLevel;

/** Add a string to the dashboard */
public class DisplayString extends DisplayableValue
{
    // Variables
    /** The value */
    protected String value_;
    /** The topic */
    protected StringTopic topic_;
    /** The publisher */
    protected StringPublisher publisher_;
    /** The subscriber */
    protected StringSubscriber subscriber_;

    // Constructor
    /**
     * Add a string to the dashboard
     *
     * @param defaultValue the default value
     * @param name the name of the variable
     * @param table the name of the table
     */
    public DisplayString(final String defaultValue, final String name, final String table)
    {
        this(defaultValue, name, table, LogLevel.COMPETITION);
    }

    /**
     * Add a string to the dashboard
     *
     * @param defaultValue the default value
     * @param name the name of the variable
     * @param table the name of the table
     * @param logLevel the log level
     */
    public DisplayString(final String defaultValue,
                         final String name,
                         final String table,
                         final LogLevel logLevel)
    {
        super(name, table, logLevel);
        value_ = defaultValue;
        if (isDisplayed_)
        {
            topic_     = NetworkTableInstance.getDefault().getTable(table_).getStringTopic(name_);
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
                        event -> { setValue(event.valueData.value.getString(), false); });
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
    public synchronized String getValue()
    {
        return value_;
    }

    // Setters
    /**
     * Set the value
     *
     * @param value the value to set
     */
    public synchronized void setValue(final String value)
    {
        setValue(value, true);
    }

    /**
     * Set the value
     *
     * @param value the value to set
     * @param publish whether or not to publish the value
     */
    public synchronized void setValue(final String value, final boolean publish)
    {
        value_ = value;
        if (publish && isDisplayed_)
        {
            publisher_.set(value_);
        }
    }

    /**
     * Registers a listener to the display values helper to update this string with the result of
     * the given supplier when the listener is called.
     *
     * @param supplier - supplier of the string to update this with
     * @return this object
     */
    public DisplayString updateWith(Supplier<String> supplier)
    {
        displayValuesHelper_.registerListener(() -> setValue(supplier.get()));
        return this;
    }
}
