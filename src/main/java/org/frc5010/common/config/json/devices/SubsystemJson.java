package org.frc5010.common.config.json.devices;

import com.fasterxml.jackson.core.exc.StreamReadException;
import com.fasterxml.jackson.databind.DatabindException;
import java.io.File;
import java.io.IOException;
import java.util.List;
import org.frc5010.common.arch.GenericSubsystem;

/** The base JSON class for subsystem configurations */
public class SubsystemJson {
  public static class DeviceEntry {
    public String device;
    public String file;

    public DeviceEntry() {}
  }
  /** A map of device names and device configuration file names */
  public List<DeviceEntry> devices;
  /** Whether to display the subsystem in the dashboard */
  public boolean display = false;
  /** The logging level for the robot */
  public String logLevel = "COMPETITION";

  /**
   * Configures the subsystem by reading device configuration files and adding devices to the
   * system.
   *
   * @param system the system being configured
   * @param directory the directory containing device configuration files
   * @throws StreamReadException if a configuration file cannot be read
   * @throws DatabindException if a configuration file cannot be parsed
   * @throws IOException if an I/O error occurs when reading a configuration file
   */
  public void configureSubsystem(GenericSubsystem system, File directory)
      throws StreamReadException, DatabindException, IOException {
    for (DeviceEntry key : devices) {
      File deviceFile = new File(directory, key.file);
      assert deviceFile.exists();
      DeviceConfigReader.readDeviceConfig(system, deviceFile, key.device);
    }
  }
}
