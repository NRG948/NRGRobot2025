/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public interface MotorController extends edu.wpi.first.wpilibj.motorcontrol.MotorController {
  /** Returns the motor controller's relative encoder. */
  RelativeEncoder getEncoder();
}
