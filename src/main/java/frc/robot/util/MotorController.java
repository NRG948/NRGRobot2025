/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public interface MotorController extends edu.wpi.first.wpilibj.motorcontrol.MotorController {
  /**
   * Creates a new motor controller that is configured to follow this motor controller.
   *
   * @param deviceID The device ID of the new motor controller.
   * @param isInvertedFromLeader Whether the new motor controller should be inverted in direction
   *     from this motor controller. This is useful for when the follower is mechanically connected
   *     to the leader but must spin in the opposite direction.
   */
  MotorController createFollower(int deviceID, boolean isInvertedFromLeader);

  /** Returns the motor controller's relative encoder. */
  RelativeEncoder getEncoder();

  /** Returns the forward limit switch. */
  LimitSwitch getForwardLimitSwitch();

  /** Returns the reverse limit switch. */
  LimitSwitch getReverseLimitSwitch();

  /** Sets the motor behavior when idle (i.e. brake or coast mode). */
  void setIdleMode(MotorIdleMode idleMode);

  /** Logs motor-specific telemetry to the data log. */
  void logTelemetry();
}
