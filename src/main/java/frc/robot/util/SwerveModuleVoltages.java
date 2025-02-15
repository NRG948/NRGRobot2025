/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

/** Represents motor voltages for one swerve module. */
public final class SwerveModuleVoltages {
  /** The drive motor voltage. */
  public final double driveVoltage;

  /** The steering motor voltage. */
  public final double steeringVoltage;

  /**
   * Creates a new SwerveModuleVoltages.
   *
   * @param driveVoltage The drive motor voltage.
   * @param steeringVoltage The steering motor voltage.
   */
  public SwerveModuleVoltages(double driveVoltage, double steeringVoltage) {
    this.driveVoltage = driveVoltage;
    this.steeringVoltage = steeringVoltage;
  }
}
