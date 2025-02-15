/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

/** Represents swerve module velocities. */
public final class SwerveModuleVelocities {
  /** The translational velocity of the wheel in m/s. */
  public double driveVelocity;

  /** The rotational velocity of the wheel in rad/s */
  public double steeringVelocity;

  /** Creates a new SwerveModuleVelocities. */
  public SwerveModuleVelocities() {
    this(0, 0);
  }

  /**
   * Creates a new SwerveModuleVelocities.
   *
   * @param driveVelocity The translational velocity of the wheel in m/s.
   * @param steeringVelocity The rotational velocity of the wheel in rad/s.
   */
  public SwerveModuleVelocities(double driveVelocity, double steeringVelocity) {
    this.driveVelocity = driveVelocity;
    this.steeringVelocity = steeringVelocity;
  }
}
