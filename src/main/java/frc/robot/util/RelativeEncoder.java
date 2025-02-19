/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public interface RelativeEncoder {
  /**
   * Sets the current position.
   *
   * <p>The unit of measure depends on the mechanism. For a mechanism that produces linear motion,
   * the unit is typically in meters. For a mechanism that produces rotational motion, the unit is
   * typically in radians.
   */
  void setPosition(double position);

  /**
   * Returns the current position.
   *
   * <p>The unit of measure depends on the mechanism. For a mechanism that produces linear motion,
   * the unit is typically in meters. For a mechanism that produces rotational motion, the unit is
   * typically in radians.
   */
  double getPosition();

  /**
   * Returns the current velocity.
   *
   * <p>The unit of measure depends on the mechanism. For a mechanism that produces linear motion,
   * the unit is typically in meters per second. For a mechanism that produces rotational motion,
   * the unit is typically in radians per second.
   */
  double getVelocity();

  /** Resets the encoder. */
  void reset();
}
