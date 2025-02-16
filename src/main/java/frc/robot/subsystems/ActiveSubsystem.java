/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import frc.robot.util.MotorIdleMode;

/** Interface representing an active subsystem. */
public interface ActiveSubsystem {
  /** Disables the subsystem. */
  void disable();

  /** Sets idle mode of motor(s) to either brake mode or coast mode. */
  void setIdleMode(MotorIdleMode idleMode);
}
