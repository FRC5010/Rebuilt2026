package org.frc5010.common.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import java.util.HashMap;
import java.util.Map;
import org.frc5010.common.arch.GenericRobot.LogLevel;
import org.frc5010.common.arch.GenericSubsystem;

public class PowerDistribution5010 extends GenericSubsystem
{
    private PowerDistribution powerDistribution;
    private static final String UNREGISTERED_CURRENT = "Unregistered Current";
    private static final String TOTAL_CURRENT        = "Total Current";
    private static final String PDP_VOLTAGE          = "PDP Voltage";
    private static final String AVERAGE_PREFIX       = "Average ";

    private Map<String, Integer> channels = new HashMap<>();
    private Map<String, Double> averages  = new HashMap<>();
    private Map<String, Integer> counts   = new HashMap<>();

    private LinearFilter currentFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    private LinearFilter voltageFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public PowerDistribution5010()
    {
        powerDistribution = new PowerDistribution();
        declareInitialValues();
    }

    public PowerDistribution5010(int id, ModuleType type)
    {
        powerDistribution = new PowerDistribution(id, type);
        declareInitialValues();
    }

    private void declareInitialValues()
    {
        networkValues.declare(TOTAL_CURRENT, 0.0);
        networkValues.declare(PDP_VOLTAGE, 0.0);
        networkValues.declare(UNREGISTERED_CURRENT, 0.0);
        networkValues.declare(AVERAGE_PREFIX + TOTAL_CURRENT, 0.0);
        networkValues.declare(AVERAGE_PREFIX + PDP_VOLTAGE, 0.0);
    }

    public void registerChannel(String name, int channel)
    {
        channels.put(name, channel);
        double initialCurrent = powerDistribution.getCurrent(channel);
        networkValues.declare(name, initialCurrent);
        networkValues.declare(AVERAGE_PREFIX + name, 0.0);
    }

    public double getChannelCurrent(String name)
    {
        return networkValues.getDouble(name);
    }

    private double updateAverage(String name, double value)
    {
        averages.merge(
            name, value,
            (avg, newValue)
                -> (avg * counts.merge(name, 1, Integer::sum) + newValue) / counts.get(name));
        return averages.get(name);
    }

    @Override public void periodic()
    {
        if (LogLevel.DEBUG == DashBoard.getLoggingLevel())
        {
            double totalCurrent = powerDistribution.getTotalCurrent();
            double pdpVoltage   = powerDistribution.getVoltage();
            // TODO: Maybe add a verbosity level to the logging...cause it could be a lot
            updatePdpValues(totalCurrent, pdpVoltage);
            double accountedCurrent = updateChannelValues();
            networkValues.set(UNREGISTERED_CURRENT, totalCurrent - accountedCurrent);
        }
    }

    private void updatePdpValues(double totalCurrent, double pdpVoltage)
    {
        networkValues.set(TOTAL_CURRENT, totalCurrent);
        networkValues.set(PDP_VOLTAGE, pdpVoltage);
        networkValues.set(AVERAGE_PREFIX + TOTAL_CURRENT, currentFilter.calculate(totalCurrent));
        networkValues.set(AVERAGE_PREFIX + PDP_VOLTAGE, voltageFilter.calculate(pdpVoltage));
    }

    private double updateChannelValues()
    {
        double accountedCurrent = 0.0;
        for (Map.Entry<String, Integer> entry : channels.entrySet())
        {
            double current = powerDistribution.getCurrent(entry.getValue());
            accountedCurrent += current;
            networkValues.set(entry.getKey(), current);
            networkValues.set(AVERAGE_PREFIX + entry.getKey(),
                              updateAverage(entry.getKey(), current));
        }
        return accountedCurrent;
    }
}
