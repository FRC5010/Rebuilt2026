package org.frc5010.common.telemetry;

import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.FloatSupplier;
import java.util.EnumSet;
import org.frc5010.common.arch.GenericRobot.LogLevel;

/** Add a float to the dashboard */
public class DisplayFloat extends DisplayableValue
{
    // Variables
    /** The value */
    protected float value_;
    /** The topic */
    protected FloatTopic topic_;
    /** The publisher */
    protected FloatPublisher publisher_;
    /** The subscriber */
    protected FloatSubscriber subscriber_;

    // Constructor
    /**
     * Add a float to the dashboard
     *
     * @param defaultValue the default value
     * @param name the name of the variable
     * @param table the name of the table
     */
    public DisplayFloat(final float defaultValue, final String name, final String table)
    {
        this(defaultValue, name, table, LogLevel.COMPETITION);
    }

    /**
     * Add a float to the dashboard
     *
     * @param defaultValue the default value
     * @param name the name of the variable
     * @param table the name of the table
     * @param logLevel the log level
     */
    public DisplayFloat(final float defaultValue,
                        final String name,
                        final String table,
                        final LogLevel logLevel)
    {
        super(name, table, logLevel);
        value_ = defaultValue;
        if (isDisplayed_)
        {
            topic_     = NetworkTableInstance.getDefault().getTable(table_).getFloatTopic(name_);
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
                    event -> { setValue(event.valueData.value.getFloat(), false); });
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
    public synchronized float getValue()
    {
        return value_;
    }

    // Setters
    /**
     * Set the value
     *
     * @param value the value to set
     */
    public synchronized void setValue(final float value)
    {
        setValue(value, true);
    }

    /**
     * Set the value
     *
     * @param value the value
     * @param publish - whether or not to publish
     */
    public synchronized void setValue(final float value, final boolean publish)
    {
        value_ = value;
        if (publish && isDisplayed_)
        {
            publisher_.set(value_);
        }
    }

    /**
     * Registers a listener to the display values helper to update this float with the result of the
     * given supplier when the listener is called.
     *
     * @param supplier - supplier of the float to update this with
     * @return this object
     */
    public DisplayFloat updateWith(FloatSupplier supplier)
    {
        displayValuesHelper_.registerListener(() -> setValue(supplier.getAsFloat()));
        return this;
    }
}
