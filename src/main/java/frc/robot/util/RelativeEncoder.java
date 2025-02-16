/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public interface RelativeEncoder {
  /** Returns the current position in meters. */
  double getPosition();

  /** Returns the current velocity in meters per second. */
  double getVelocity();

  /** Resets the encoder. */
  void reset();
}
