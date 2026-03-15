package org.frc5010.common.telemetry;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import java.util.EnumSet;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericRobot.LogLevel;

/** Add a length to the dashboard */
public class DisplayLength extends DisplayableValue
{
    /** Length value */
    MutDistance length_;
    /** Length unit */
    protected final DistanceUnit unit_;
    /** The topic */
    protected DoubleTopic topic_;
    /** The publisher */
    protected DoublePublisher publisher_;
    /** The subscriber */
    protected DoubleSubscriber subscriber_;

    // Constructor
    /**
     * Add a length to the dashboard
     *
     * @param length - length
     * @param unit - unit
     * @param name - name of the variable
     * @param table - name of the table
     */
    public DisplayLength(final double length,
                         final DistanceUnit unit,
                         final String name,
                         final String table)
    {
        this(length, unit, name, table, LogLevel.COMPETITION);
    }

    /**
     * Add a length to the dashboard
     *
     * @param length - length unit in the unit
     * @param unit - unit
     * @param name - name of the variable
     * @param table - name of the table
     * @param logLevel - the log level
     */
    public DisplayLength(final double length,
                         final DistanceUnit unit,
                         final String name,
                         final String table,
                         LogLevel logLevel)
    {
        super(String.format("%s (%s)", name, unit.symbol()), table, logLevel);
        length_ = new MutDistance(length, unit.getBaseUnit().convertFrom(length, unit), unit);
        unit_   = unit;
        if (isDisplayed_)
        {
            topic_     = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
            publisher_ = topic_.publish();
            init(logLevel);
        }
    }

    // Constructor
    /**
     * Add a length to the dashboard
     *
     * @param length - length with units
     * @param name - name of the variable
     * @param table - name of the table
     */
    public DisplayLength(final Distance length, final String name, final String table)
    {
        this(length, name, table, LogLevel.COMPETITION);
    }

    /**
     * Add a length to the dashboard
     *
     * @param length - length with units
     * @param name - name of the variable
     * @param table - name of the table
     * @param logLevel - the log level
     */
    public DisplayLength(final Distance length,
                         final String name,
                         final String table,
                         LogLevel logLevel)
    {
        super(String.format("%s (%s)", name, length.unit().symbol()), table, logLevel);
        length_ = length.mutableCopy();
        unit_   = length.unit();
        if (isDisplayed_)
        {
            topic_     = NetworkTableInstance.getDefault().getTable(table_).getDoubleTopic(name_);
            publisher_ = topic_.publish();
            init(logLevel);
        }
    }

    protected void init(LogLevel logLevel)
    {
        publisher_.setDefault(length_.in(unit_));
        if (LogLevel.CONFIG == logLevel)
        {
            if (isDisplayed_)
                topic_.setPersistent(true);
            if (DisplayValuesHelper.isAtLogLevel(LogLevel.CONFIG))
            {
                subscriber_     = topic_.subscribe(length_.in(unit_));
                listenerHandle_ = NetworkTableInstance.getDefault().addListener(
                    subscriber_, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                    event -> { setLength(event.valueData.value.getDouble(), unit_, false); });
                setLength(subscriber_.get(), unit_, false);
            }
        }
    }

    // Setters

    /**
     * Sets the length using a value and a unit, and publishes the value
     *
     * @param length the value of the length to set
     * @param unit the unit of the length to set
     */
    public void setLength(final double length, final DistanceUnit unit)
    {
        setLength(length, unit, true);
    }

    /**
     * Sets the length using a value and a unit, and optionally publishes the value
     *
     * @param length the value of the length to set
     * @param unit the unit of the length to set
     * @param publish whether or not to publish the value
     */
    public void setLength(final double length, final DistanceUnit unit, final boolean publish)
    {
        setLength(unit.of(length), publish);
    }

    /**
     * Sets the length using a Distance object and publishes the value.
     *
     * @param length - the Distance object representing the length to set
     */
    public void setLength(final Distance length)
    {
        setLength(length, true);
    }

    /**
     * Sets the length using a Distance object
     *
     * @param length - the Distance object representing the length to set
     * @param publish - whether or not to publish the value
     */
    public void setLength(final Distance length, final boolean publish)
    {
        length_.mut_setBaseUnitMagnitude(length.baseUnitMagnitude());
        publish(publish);
    }

    /**
     * Sets the length using a DisplayLength object
     *
     * @param length the DisplayLength object to get the length from
     */
    public void setLength(final DisplayLength length)
    {
        setLength(length.length_, true);
    }

    /**
     * Publishes the current length to the network table if the publish flag is true.
     *
     * @param publish - flag indicating whether to publish the length
     */
    protected void publish(boolean publish)
    {
        if (publish && isDisplayed_)
        {
            publisher_.set(length_.in(unit_));
        }
    }

    /**
     * Get the length
     *
     * @return the length
     */
    public Distance getLength()
    {
        return length_;
    }

    public double getLengthInMeters()
    {
        return length_.in(Meters);
    }

    /**
     * Registers a listener to update the DisplayLength with a supplier of distances. The supplier
     * is called every time the dashboard is updated. This is useful for updating a DisplayLength
     * with a changing quantity, such as the distance from the robot to the goal.
     *
     * @param supplier the supplier of distances
     * @return this
     */
    public DisplayLength updateWith(Supplier<Distance> supplier)
    {
        displayValuesHelper_.registerListener(() -> setLength(supplier.get()));
        return this;
    }
}
