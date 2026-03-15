package org.frc5010.common.config;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.arch.GenericSubsystem;

/**
 * Interface for configuring robot devices from JSON configuration data.
 *
 * <p>Implementations of this interface define how to instantiate and configure specific robot
 * hardware devices (motors, sensors, subsystems, etc.) from JSON configuration objects. This
 * pattern allows robot configuration to be externalized to JSON files while maintaining type safety
 * and compile-time checking.
 *
 * <p>Typical usage:
 *
 * <pre>
 *   DeviceConfiguration config = objectMapper.readValue(jsonFile, ConcreteConfigClass.class);
 *   MyDevice device = (MyDevice) config.configure(subsystem);
 * </pre>
 *
 * @see GenericSubsystem
 * @see SubsystemBase
 */
public interface DeviceConfiguration {
    /**
     * Configures and returns a device instance based on this configuration object.
     *
     * <p>This method is called with the subsystem that will contain the device. The implementing
     * class is responsible for creating the appropriate device instance using the configuration
     * data and adding it to the subsystem if necessary.
     *
     * @param deviceHandler the {@link SubsystemBase} to configure with this device configuration
     * @return the configured device object, or {@code null} if no device was created
     */
    public default Object configure(SubsystemBase deviceHandler)
    {
        return null;
    }

    /**
     * Configures and returns a device instance using a {@link GenericSubsystem}.
     *
     * <p>This method delegates to {@link #configure(SubsystemBase)} after casting the {@code
     * GenericSubsystem} to a {@code SubsystemBase}. Subclasses can override this method for more
     * specific handling when working with {@code GenericSubsystem} instances.
     *
     * @param deviceHandler the {@link GenericSubsystem} to configure with this device configuration
     * @return the configured device object, or {@code null} if no device was created
     */
    public default Object configure(GenericSubsystem deviceHandler)
    {
        return configure((SubsystemBase) deviceHandler);
    }
}
