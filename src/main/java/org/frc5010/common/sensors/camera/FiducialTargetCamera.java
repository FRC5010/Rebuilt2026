// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.sensors.camera;

import java.util.List;

/** Add your docs here. */
public interface FiducialTargetCamera {
    /**
     * Gets the current list of fiducial IDs for this camera.
     *
     * @return the current list of fiducial IDs
     */
    public List<Integer> getFiducialIds();

    /**
     * Sets the list of fiducial IDs for this camera. The camera will only consider targets with IDs
     * in this list when locating targets. This does not change the list spefified at construction
     * time to the pose camera so that the camera can be both a pose and target camera.
     *
     * @param fiducialIds the list of fiducial IDs to consider
     */
    public void setFiducialIds(List<Integer> fiducialIds);
}
