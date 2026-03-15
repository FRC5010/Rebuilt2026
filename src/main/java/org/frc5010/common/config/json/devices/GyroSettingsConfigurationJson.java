package org.frc5010.common.config.json.devices;

import com.studica.frc.AHRS.NavXComType;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.sensors.gyro.NavXGyro;
import org.frc5010.common.sensors.gyro.PigeonGyro;
import org.frc5010.common.sensors.gyro.YagslGyro;

/** Instatiates a Gyro sensor based on the specified type */
public class GyroSettingsConfigurationJson implements DeviceConfiguration
{
    /** The type of gyro sensor */
    public String type;
    /** The ID of the gyro sensor, if needed */
    public int id;
    /** Whether the gyro sensor is inverted */
    public boolean inverted = false;
    /** The CAN bus the gyro is connected to */
    public String canbus = "";

    /**
     * Configures a gyro sensor based on the specified type.
     *
     * @param deviceHandler the GenericSubsystem to configure
     * @return the configured gyro sensor, or null if the type is not recognized
     */
    @Override public Object configure(GenericSubsystem deviceHandler)
    {
        switch (type)
        {
            case "navx":
                return new NavXGyro(NavXComType.kMXP_SPI);
            case "pigeon2":
                return new PigeonGyro(id);
            case "yagsl":
                return new YagslGyro();
            default:
                return null;
        }
    }
}
