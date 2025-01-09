/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

/** An enum representing a swerve module angle encoder. */
public enum SwerveAngleEncoder {
  FrontLeft,
  FrontRight,
  BackLeft,
  BackRight;

  /** Returns the index of the CANcoder ids. */
  public int getIndex() {
    return this.ordinal();
  }
}
