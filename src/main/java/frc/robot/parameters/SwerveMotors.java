/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

/** An enum representing a swerve module motor. */
public enum SwerveMotors {
  FrontLeftDrive,
  FrontLeftSteering,
  FrontRightDrive,
  FrontRightSteering,
  BackLeftDrive,
  BackLeftSteering,
  BackRightDrive,
  BackRightSteering;

  /** Returns the index of the Talon CAN ids. */
  public int getIndex() {
    return this.ordinal();
  }
}
