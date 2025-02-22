/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

/** An interface for absolute angle encoders. */
public interface AbsoluteAngleEncoder {
  /** Returns the angle in with range -π to π radians */
  double getAngle();
}
