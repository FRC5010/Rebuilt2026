// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.telemetry;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.arch.WpiHelperInterface;

/** Registers values with the network tables */
public class DisplayValuesHelper implements WpiHelperInterface
{
    protected ShuffleboardTab tab;
    protected ShuffleboardLayout layout;
    protected String tabName              = "frc";
    protected Optional<String> layoutName = Optional.empty();
    protected int column                  = 0;
    protected boolean isDisplayed;
    protected LogLevel logLevel = LogLevel.COMPETITION;

    protected final List<Runnable> listeners         = new ArrayList<>();
    protected final List<DisplayableValue> internals = new ArrayList<>();

    /**
     * Constructs a DisplayValuesHelper with the specified tab and table names, display state, and
     * starting column.
     *
     * @param tab the name of the Shuffleboard tab
     */
    public DisplayValuesHelper(String tab)
    {
        this(tab, Optional.empty(), false, 0);
    }

    /**
     * Constructs a DisplayValuesHelper with the specified tab and table names, display state, and
     * starting column.
     *
     * @param tab the name of the Shuffleboard tab
     */
    public DisplayValuesHelper(String tab, String table)
    {
        this(tab, Optional.ofNullable(table), false, 0);
    }

    /**
     * Constructs a DisplayValuesHelper with the specified tab and table names, and display state.
     * The starting column defaults to 0.
     *
     * @param tab the name of the Shuffleboard tab
     * @param table the name of the Shuffleboard layout
     * @param isDisplayed whether the values should be displayed on the dashboard
     */
    public DisplayValuesHelper(String tab, String table, boolean isDisplayed)
    {
        this(tab, Optional.ofNullable(table), isDisplayed, 0);
    }

    /**
     * Constructs a DisplayValuesHelper with the specified tab and table names, display state, and
     * starting column.
     *
     * @param tab the name of the Shuffleboard tab
     * @param table the name of the Shuffleboard layout
     * @param isDisplayed whether the values should be displayed on the dashboard
     * @param startingColumn the starting column for the layout
     */
    public DisplayValuesHelper(String tab,
                               Optional<String> table,
                               boolean isDisplayed,
                               int startingColumn)
    {
        column           = startingColumn;
        this.isDisplayed = isDisplayed;
        tabName          = tab;
        layoutName       = table;
        if (!isDisplayed)
            return;

        makeDisplayed();
    }

    /**
     * Sets the display state to active, initializes the Shuffleboard tab and layout for displaying
     * values. Configures the layout size and position based on the provided column.
     */
    public void makeDisplayed()
    {
        isDisplayed = true;
        this.tab    = Shuffleboard.getTab(tabName);
        this.layout = this.tab.getLayout(layoutName.orElse("values"), BuiltInLayouts.kList)
                          .withSize(2, 4)
                          .withPosition(column, 0);
    }

    /**
     * Registers a listener to the display values helper. Listeners will be called when the
     * notifyListeners function is called. This function is used to update the display values on the
     * dashboard.
     *
     * @param listener - Runnable to be called when notifyListeners is called
     */
    public void registerListener(Runnable listener)
    {
        listeners.add(listener);
    }

    /**
     * Removes a listener from the display values helper. This listener will not be called when
     * notifyListeners is called.
     *
     * @param listener - Runnable to be removed from the list of listeners
     */
    public void unregisterListener(Runnable listener)
    {
        listeners.remove(listener);
    }

    /**
     * Calls all registered listeners. This should be called when a value is updated so that all
     * dashboard values are updated.
     */
    public void notifyListeners()
    {
        for (Runnable listener : listeners)
        {
            listener.run();
        }
    }

    /**
     * Registers a value to be displayed on the dashboard. The value will be retrieved by calling
     * the provided Supplier, and the result will be displayed on the dashboard when the
     * notifyListeners function is called.
     *
     * @param key the name of the value to be displayed
     * @param value a Supplier that returns the value to be displayed
     */
    public void display(String key, Supplier<String> value)
    {
        internals.add(new DisplayString("", key, getNtFolder()).setDisplayValuesHelper(this));
        registerListener(value::get);
    }

    public void display(String key, Sendable value)
    {
        SmartDashboard.putData(tabName + "/" + layoutName.map(it -> it + "/").orElse("") + key,
                               value);
    }

    /**
     * Sets the logging level for the display. Values that are at a higher or equal level to the
     * specified level will be displayed on the dashboard.
     *
     * @param level the level to set the display to
     */
    public void setLoggingLevel(LogLevel level)
    {
        logLevel = level;
    }

    /**
     * Gets the current logging level of the display.
     *
     * @return the LogLevel that the display is currently set to
     */
    public LogLevel getLoggingLevel()
    {
        return logLevel;
    }

    /**
     * Advances the column number for the next value to be placed in.
     *
     * @param name the name of the next column
     */
    public void nextColumn(String name)
    {
        layoutName = Optional.ofNullable(name);
        if (!isDisplayed)
            return;
        column += 2;
        layout = tab.getLayout(name, BuiltInLayouts.kList).withSize(2, 4).withPosition(column, 0);
    }

    /**
     * Constructs the NetworkTable folder path for the current Shuffleboard layout.
     *
     * @return a String representing the path in the format "Shuffleboard/<tabTitle>/<layoutTitle>"
     */
    private String getNtFolder()
    {
        if (!isDisplayed)
            return "SmartDashboard/" + tabName + "/" + layoutName.map(it -> it).orElse("");
        return "Shuffleboard/" + tabName + "/" + layoutName.map(it -> it).orElse("");
    }

    /**
     * Checks if the robot's current logging level is at or above the specified log level.
     *
     * @param logLevel the log level to check against
     * @return true if the robot's current log level is at or above the specified level, false
     *     otherwise
     */
    public static boolean isAtLogLevel(LogLevel logLevel)
    {
        switch (logLevel)
        {
            case DEBUG:
            {
                return logLevel == LogLevel.DEBUG || logLevel == LogLevel.INFO;
            }
            case INFO:
            {
                return logLevel == LogLevel.INFO || logLevel == LogLevel.DEBUG;
            }
            case CONFIG:
            {
                return logLevel == LogLevel.CONFIG || logLevel == LogLevel.DEBUG;
            }
            case COMPETITION:
            {
                return true;
            }
            default:
            {
                return true;
            }
        }
    }

    /**
     * Constructs a DisplayAngle object with the given name and units, and registers it with
     * NetworkTables.
     *
     * @param name the name of the angle to be displayed
     * @return a DisplayAngle object
     */
    public DisplayAngle makeDisplayAngle(String name)
    {
        DisplayAngle angle = new DisplayAngle(0, Degrees, name, getNtFolder());
        angle.setDisplayValuesHelper(this);
        return angle;
    }

    /**
     * Constructs a DisplayAngle object with the given name and units, and registers it with
     * NetworkTables, at the INFO logging level.
     *
     * @param name the name of the angle to be displayed
     * @return a DisplayAngle object
     */
    public DisplayAngle makeInfoAngle(String name)
    {
        DisplayAngle angle = new DisplayAngle(0, Degrees, name, getNtFolder(), LogLevel.INFO);
        angle.setDisplayValuesHelper(this);
        return angle;
    }

    /**
     * Constructs a DisplayAngle object with the given name and units, and registers it with
     * NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the angle to be displayed
     * @return a DisplayAngle object
     */
    public DisplayAngle makeConfigAngle(String name)
    {
        DisplayAngle angle = new DisplayAngle(0, Degrees, name, getNtFolder(), LogLevel.CONFIG);
        angle.setDisplayValuesHelper(this);
        return angle;
    }

    /**
     * Constructs a DisplayLength object with the given name and units, and registers it with
     * NetworkTables.
     *
     * @param name the name of the length to be displayed
     * @return a DisplayLength object
     */
    public DisplayLength makeDisplayLength(String name)
    {
        DisplayLength length = new DisplayLength(0, Meters, name, getNtFolder());
        length.setDisplayValuesHelper(this);
        return length;
    }

    /**
     * Constructs a DisplayLength object with the given name and units, and registers it with
     * NetworkTables, at the INFO logging level.
     *
     * @param name the name of the length to be displayed
     * @return a DisplayLength object
     */
    public DisplayLength makeInfoLength(String name)
    {
        DisplayLength length = new DisplayLength(0, Meters, name, getNtFolder(), LogLevel.INFO);
        length.setDisplayValuesHelper(this);
        return length;
    }

    /**
     * Constructs a DisplayLength object with the given name and units, and registers it with
     * NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the length to be displayed
     * @return a DisplayLength object
     */
    public DisplayLength makeConfigLength(String name)
    {
        DisplayLength length = new DisplayLength(0, Meters, name, getNtFolder(), LogLevel.CONFIG);
        length.setDisplayValuesHelper(this);
        return length;
    }

    /**
     * Constructs a DisplayTime object with the given name and units, and registers it with
     * NetworkTables.
     *
     * @param name the name of the time to be displayed
     * @return a DisplayTime object
     */
    public DisplayTime makeDisplayTime(String name)
    {
        DisplayTime time = new DisplayTime(0, Seconds, name, getNtFolder());
        time.setDisplayValuesHelper(this);
        return time;
    }

    /**
     * Constructs a DisplayTime object with the given name and units, and registers it with
     * NetworkTables, at the INFO logging level.
     *
     * @param name the name of the time to be displayed
     * @return a DisplayTime object
     */
    public DisplayTime makeInfoTime(String name)
    {
        DisplayTime time = new DisplayTime(0, Seconds, name, getNtFolder(), LogLevel.INFO);
        time.setDisplayValuesHelper(this);
        return time;
    }

    /**
     * Constructs a DisplayTime object with the given name and units, and registers it with
     * NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the time to be displayed
     * @return a DisplayTime object
     */
    public DisplayTime makeConfigTime(String name)
    {
        DisplayTime time = new DisplayTime(0, Seconds, name, getNtFolder(), LogLevel.CONFIG);
        time.setDisplayValuesHelper(this);
        return time;
    }

    /**
     * Constructs a DisplayVoltage object with the given name and units, and registers it with
     * NetworkTables.
     *
     * @param name the name of the voltage to be displayed
     * @return a DisplayVoltage object
     */
    public DisplayVoltage makeDisplayVoltage(String name)
    {
        DisplayVoltage voltage = new DisplayVoltage(0, Volts, name, getNtFolder());
        voltage.setDisplayValuesHelper(this);
        return voltage;
    }

    /**
     * Constructs a DisplayVoltage object with the given name and units, and registers it with
     * NetworkTables, at the INFO logging level.
     *
     * @param name the name of the voltage to be displayed
     * @return a DisplayVoltage object
     */
    public DisplayVoltage makeInfoVoltage(String name)
    {
        DisplayVoltage voltage = new DisplayVoltage(0, Volts, name, getNtFolder(), LogLevel.INFO);
        voltage.setDisplayValuesHelper(this);
        return voltage;
    }

    /**
     * Constructs a DisplayVoltage object with the given name and units, and registers it with
     * NetworkTables at the CONFIG logging level.
     *
     * @param name the name of the voltage to be displayed
     * @return a DisplayVoltage object
     */
    public DisplayVoltage makeConfigVoltage(String name)
    {
        DisplayVoltage voltage = new DisplayVoltage(0, Volts, name, getNtFolder(), LogLevel.CONFIG);
        voltage.setDisplayValuesHelper(this);
        return voltage;
    }

    /**
     * Constructs a DisplayBoolean object with the given name and registers it with NetworkTables.
     *
     * @param name the name of the boolean to be displayed
     * @return a DisplayBoolean object
     */
    public DisplayBoolean makeDisplayBoolean(String name)
    {
        DisplayBoolean booleanValue = new DisplayBoolean(false, name, getNtFolder());
        booleanValue.setDisplayValuesHelper(this);
        return booleanValue;
    }

    /**
     * Constructs a DisplayBoolean object with the given name and registers it with NetworkTables at
     * the INFO logging level.
     *
     * @param name the name of the boolean to be displayed
     * @return a DisplayBoolean object
     */
    public DisplayBoolean makeInfoBoolean(String name)
    {
        DisplayBoolean booleanValue = new DisplayBoolean(false, name, getNtFolder(), LogLevel.INFO);
        booleanValue.setDisplayValuesHelper(this);
        return booleanValue;
    }

    /**
     * Constructs a DisplayBoolean object with the given name and registers it with NetworkTables at
     * the CONFIG logging level.
     *
     * @param name the name of the boolean to be displayed
     * @return a DisplayBoolean object
     */
    public DisplayBoolean makeConfigBoolean(String name)
    {
        DisplayBoolean booleanValue =
            new DisplayBoolean(false, name, getNtFolder(), LogLevel.CONFIG);
        booleanValue.setDisplayValuesHelper(this);
        return booleanValue;
    }

    /**
     * Constructs a DisplayDouble object with the given name and registers it with NetworkTables.
     *
     * @param name the name of the double to be displayed
     * @return a DisplayDouble object
     */
    public DisplayDouble makeDisplayDouble(String name)
    {
        DisplayDouble doubleValue = new DisplayDouble(0, name, getNtFolder());
        doubleValue.setDisplayValuesHelper(this);
        return doubleValue;
    }

    /**
     * Constructs a DisplayDouble object with the given name and registers it with NetworkTables at
     * the INFO logging level.
     *
     * @param name the name of the double to be displayed
     * @return a DisplayDouble object
     */
    public DisplayDouble makeInfoDouble(String name)
    {
        DisplayDouble doubleValue = new DisplayDouble(0, name, getNtFolder(), LogLevel.INFO);
        doubleValue.setDisplayValuesHelper(this);
        return doubleValue;
    }

    /**
     * Constructs a DisplayDouble object with the given name and registers it with NetworkTables at
     * the CONFIG logging level.
     *
     * @param name the name of the double to be displayed
     * @return a DisplayDouble object
     */
    public DisplayDouble makeConfigDouble(String name)
    {
        DisplayDouble doubleValue = new DisplayDouble(0, name, getNtFolder(), LogLevel.CONFIG);
        doubleValue.setDisplayValuesHelper(this);
        return doubleValue;
    }

    /**
     * Constructs a DisplayString object with the given name and registers it with NetworkTables.
     *
     * @param name the name of the string to be displayed
     * @return a DisplayString object
     */
    public DisplayString makeDisplayString(String name)
    {
        DisplayString stringValue = new DisplayString("", name, getNtFolder());
        stringValue.setDisplayValuesHelper(this);
        return stringValue;
    }

    /**
     * Constructs a DisplayString object with the given name and registers it with NetworkTables at
     * the INFO logging level.
     *
     * @param name the name of the string to be displayed
     * @return a DisplayString object
     */
    public DisplayString makeInfoString(String name)
    {
        DisplayString stringValue = new DisplayString("", name, getNtFolder(), LogLevel.INFO);
        stringValue.setDisplayValuesHelper(this);
        return stringValue;
    }

    /**
     * Constructs a DisplayString object with the given name and registers it with NetworkTables at
     * the CONFIG logging level.
     *
     * @param name the name of the string to be displayed
     * @return a DisplayString object
     */
    public DisplayString makeConfigString(String name)
    {
        DisplayString stringValue = new DisplayString("", name, getNtFolder(), LogLevel.CONFIG);
        stringValue.setDisplayValuesHelper(this);
        return stringValue;
    }

    /**
     * Constructs a DisplayLong object with the given name and registers it with NetworkTables.
     *
     * @param name the name of the long to be displayed
     * @return a DisplayLong object
     */
    public DisplayLong makeDisplayLong(String name)
    {
        DisplayLong longValue = new DisplayLong(0, name, getNtFolder());
        longValue.setDisplayValuesHelper(this);
        return longValue;
    }

    /**
     * Constructs a DisplayLong object with the given name and registers it with NetworkTables at
     * the INFO logging level.
     *
     * @param name the name of the long to be displayed
     * @return a DisplayLong object
     */
    public DisplayLong makeInfoLong(String name)
    {
        DisplayLong longValue = new DisplayLong(0, name, getNtFolder(), LogLevel.INFO);
        longValue.setDisplayValuesHelper(this);
        return longValue;
    }

    /**
     * Constructs a DisplayLong object with the given name and registers it with NetworkTables at
     * the CONFIG logging level.
     *
     * @param name the name of the long to be displayed
     * @return a DisplayLong object
     */
    public DisplayLong makeConfigLong(String name)
    {
        DisplayLong longValue = new DisplayLong(0, name, getNtFolder(), LogLevel.CONFIG);
        longValue.setDisplayValuesHelper(this);
        return longValue;
    }

    /**
     * Constructs a DisplayFloat object with the given name and registers it with NetworkTables.
     *
     * @param name the name of the float to be displayed
     * @return a DisplayFloat object
     */
    public DisplayFloat makeDisplayFloat(String name)
    {
        DisplayFloat floatValue = new DisplayFloat(0, name, getNtFolder());
        floatValue.setDisplayValuesHelper(this);
        return floatValue;
    }

    /**
     * Constructs a DisplayFloat object with the given name and registers it with NetworkTables at
     * the INFO logging level.
     *
     * @param name the name of the float to be displayed
     * @return a DisplayFloat object
     */
    public DisplayFloat makeInfoFloat(String name)
    {
        DisplayFloat floatValue = new DisplayFloat(0, name, getNtFolder(), LogLevel.INFO);
        floatValue.setDisplayValuesHelper(this);
        return floatValue;
    }

    /**
     * Constructs a DisplayFloat object with the given name and registers it with NetworkTables at
     * the CONFIG logging level.
     *
     * @param name the name of the float to be displayed
     * @return a DisplayFloat object
     */
    public DisplayFloat makeConfigFloat(String name)
    {
        DisplayFloat floatValue = new DisplayFloat(0, name, getNtFolder(), LogLevel.CONFIG);
        floatValue.setDisplayValuesHelper(this);
        return floatValue;
    }

    /**
     * Constructs a DisplayCurrent object with the given name and registers it with NetworkTables.
     *
     * @param name the name of the current to be displayed
     * @return a DisplayCurrent object
     */
    public DisplayCurrent makeDisplayCurrent(String name)
    {
        DisplayCurrent currentValue = new DisplayCurrent(0, Amps, name, getNtFolder());
        currentValue.setDisplayValuesHelper(this);
        return currentValue;
    }

    /**
     * Constructs a DisplayCurrent object with the given name and registers it with NetworkTables at
     * the INFO logging level.
     *
     * @param name the name of the current to be displayed
     * @return a DisplayCurrent object
     */
    public DisplayCurrent makeInfoCurrent(String name)
    {
        DisplayCurrent currentValue =
            new DisplayCurrent(0, Amps, name, getNtFolder(), LogLevel.INFO);
        currentValue.setDisplayValuesHelper(this);
        return currentValue;
    }

    /**
     * Constructs a DisplayCurrent object with the given name and registers it with NetworkTables at
     * the CONFIG logging level.
     *
     * @param name the name of the current to be displayed
     * @return a DisplayCurrent object
     */
    public DisplayCurrent makeConfigCurrent(String name)
    {
        DisplayCurrent currentValue =
            new DisplayCurrent(0, Amps, name, getNtFolder(), LogLevel.CONFIG);
        currentValue.setDisplayValuesHelper(this);
        return currentValue;
    }
}
