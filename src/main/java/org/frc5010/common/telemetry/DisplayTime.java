package org.frc5010.common.telemetry;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.Time;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericRobot.LogLevel;

/** Add a time to the dashboard */
public class DisplayTime extends DisplayableValue
{
    /** The time value */
    final MutTime time_;
    /** The time unit */
    protected final TimeUnit unit_;
    /** The topic */
    protected DoubleTopic topic_;
    /** The publisher */
    protected DoublePublisher publisher_;
    /** The subscriber */
    protected DoubleSubscriber subscriber_;

    // Constructors
    /**
     * Add a time to the dashboard
     *
     * @param unit - time unit
     * @param unitTime - time in that unit
     * @param name - name of the time
     * @param table - name of the table
     */
    public DisplayTime(final double unitTime,
                       final TimeUnit unit,
                       final String name,
                       final String table)
    {
        this(unitTime, unit, name, table, LogLevel.COMPETITION);
    }

    /**
     * Add a time to the dashboard
     *
     * @param unit - time unit
     * @param unitTime - time in that unit
     * @param name - name of the time
     * @param table - name of the table
     * @param logLevel - the log level
     */
    public DisplayTime(final double unitTime,
                       final TimeUnit unit,
                       final String name,
                       final String table,
                       final LogLevel logLevel)
    {
        super(String.format("%s (%s)", name, unit.symbol()), table, logLevel);
        time_ = new MutTime(unitTime, unit.getBaseUnit().convertFrom(unitTime, unit), unit);
        unit_ = unit;
        if (isDisplayed_)
        {
            topic_     = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
            publisher_ = topic_.publish();
            init(logLevel);
        }
    }

    /**
     * Add a time to the dashboard
     *
     * @param unitTime - time in that unit
     * @param name - name of the time
     * @param table - name of the table
     */
    public DisplayTime(final Time unitTime, final String name, final String table)
    {
        this(unitTime, name, table, LogLevel.COMPETITION);
    }

    /**
     * Add a time to the dashboard
     *
     * @param unitTime
     * @param name
     * @param table
     * @param logLevel
     */
    public DisplayTime(final Time unitTime,
                       final String name,
                       final String table,
                       final LogLevel logLevel)
    {
        super(String.format("%s (%s)", name, unitTime.unit().symbol()), table, logLevel);
        time_ = unitTime.mutableCopy();
        unit_ = unitTime.unit();
        if (isDisplayed_)
        {
            topic_     = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
            publisher_ = topic_.publish();
            init(logLevel);
        }
    }

    protected void init(LogLevel logLevel)
    {
        if (LogLevel.CONFIG == logLevel)
        {
            if (isDisplayed_)
                topic_.setPersistent(true);
            if (DisplayValuesHelper.isAtLogLevel(LogLevel.CONFIG))
            {
                subscriber_     = topic_.subscribe(time_.in(unit_));
                listenerHandle_ = NetworkTableInstance.getDefault().addListener(
                    subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    event -> { setTime(event.valueData.value.getDouble(), unit_, false); });
                setTime(subscriber_.get(), unit_, false);
            }
        }
        publisher_.setDefault(time_.in(unit_));
    }

    // Setters

    /**
     * Sets the time
     *
     * @param time - time in the given unit
     * @param unit - time unit
     * @see #setTime(double, TimeUnit, boolean)
     */
    public void setTime(final double time, final TimeUnit unit)
    {
        setTime(time, unit, true);
    }

    /**
     * Sets the time
     *
     * @param time - time in the given unit
     * @param unit - time unit
     * @param publish - whether or not to publish
     */
    public void setTime(final double time, final TimeUnit unit, final boolean publish)
    {
        setTime(unit.of(time), publish);
    }

    /**
     * Sets the time
     *
     * @param time - the Time object representing the time to set
     * @param publish - whether or not to publish
     */
    public void setTime(final Time time, final boolean publish)
    {
        time_.mut_setBaseUnitMagnitude(time.baseUnitMagnitude());
        publish(publish);
    }

    /**
     * Sets the time using a Time object and publishes the value.
     *
     * @param time - the Time object representing the time to set
     */
    public void setTime(final Time time)
    {
        setTime(time, true);
    }

    /**
     * Sets the angle using a {@link DisplayTime} object and publishes the value.
     *
     * @param time - the {@link DisplayTime} object representing the time to set
     */
    public void setTime(final DisplayTime time)
    {
        setTime(time.time_, true);
    }

    /**
     * Publishes the current time to the network table if the publish flag is true.
     *
     * @param publish - flag indicating whether to publish the time
     */
    protected void publish(boolean publish)
    {
        if (publish && isDisplayed_)
        {
            publisher_.set(time_.in(unit_));
        }
    }

    /**
     * Gets the current time
     *
     * @return the current time
     */
    public Time getTime()
    {
        return time_;
    }

    public double getTimeInSeconds()
    {
        return time_.in(Seconds);
    }

    /**
     * Registers a listener to update the DisplayTime with a supplier of times. The supplier is
     * called every time the dashboard is updated. This is useful for updating a DisplayTime with a
     * changing quantity, such as the time from the beginning of a match.
     *
     * @param supplier the supplier of times
     * @return this
     */
    public DisplayTime updateWith(Supplier<Time> supplier)
    {
        displayValuesHelper_.registerListener(() -> setTime(supplier.get()));
        return this;
    }
}
