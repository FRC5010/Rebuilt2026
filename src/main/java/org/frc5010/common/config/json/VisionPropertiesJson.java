// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.Map;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.vision.AprilTags;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/** JSON class with an array of cameras to configure */
public class VisionPropertiesJson {
  /** An array of camera names */
  public String[] cameras;

  public String aprilTagLayout = "default";
  public String simulatedField = "default";
  public Map<String, String[]> gamePieces = new HashMap<>();
  public boolean viewGamePieces = true;

  /**
   * Creates cameras for a given robot using the provided map of camera configurations.
   *
   * @param robot the robot to add the cameras to
   * @param map a map of camera configurations, where the key is the camera name and the value is
   *     the configuration object
   */
  public void createCameraSystem(GenericRobot robot, Map<String, CameraConfigurationJson> map) {
    map.keySet()
        .forEach(
            it -> {
              map.get(it).configureCamera(robot);
            });
  }

  /**
   * Reads in cameras from the provided directory.
   *
   * @param directory the directory to read from
   * @return the map of camera configurations
   */
  public Map<String, CameraConfigurationJson> readCameraSystem(File directory) throws IOException {
    if (!simulatedField.equalsIgnoreCase("default")) {
      try {
        SimulatedArena arena =
            Class.forName(simulatedField)
                .asSubclass(SimulatedArena.class)
                .getDeclaredConstructor()
                .newInstance();
        if (!gamePieces.isEmpty()) {
          arena.clearGamePieces();
          for (String key : gamePieces.keySet()) {
            arena.addGamePiece(
                Class.forName(gamePieces.get(key)[0])
                    .asSubclass(GamePieceOnFieldSimulation.class)
                    .getDeclaredConstructor()
                    .newInstance());
          }
        }
        SimulatedArena.overrideInstance(arena);
      } catch (ClassNotFoundException
          | NoSuchMethodException
          | InstantiationException
          | IllegalAccessException
          | InvocationTargetException e) {
        System.err.println("Error creating arena instance: " + e.getMessage());
        e.printStackTrace();
        throw new RuntimeException(e);
      }
    }
    switch (aprilTagLayout) {
      case "5010" -> AprilTags.setAprilTagFieldLayout(AprilTags.aprilTagRoomLayout);
      case "default" -> {
        AprilTagFieldLayout layout =
            AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        AprilTags.setAprilTagFieldLayout(layout);
      }
      default -> {
        AprilTagFieldLayout layout;
        try {
          layout =
              AprilTagFieldLayout.loadFromResource(
                  AprilTagFields.valueOf(aprilTagLayout).m_resourceFile);
          AprilTags.setAprilTagFieldLayout(layout);
        } catch (IllegalArgumentException e) {
          layout = AprilTagFieldLayout.loadFromResource(aprilTagLayout);
          AprilTags.setAprilTagFieldLayout(layout);
        }
      }
    }

    Map<String, CameraConfigurationJson> camerasMap = new HashMap<>();
    for (String cameraString : cameras) {
      File cameraFile = new File(directory, "cameras/" + cameraString);
      assert cameraFile.exists();
      CameraConfigurationJson camera =
          new ObjectMapper().readValue(cameraFile, CameraConfigurationJson.class);
      camera.setViewGamePieces(viewGamePieces);
      camerasMap.put(camera.name, camera);
    }
    return camerasMap;
  }
}
