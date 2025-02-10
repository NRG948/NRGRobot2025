/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.subsystems.Elevator;

public enum ElevatorLevel {
  // TODO change pivotOffsets
  STOWED(
      Elevator.MIN_HEIGHT,
      Math.toRadians(90),
      0), // TO-DO: Determine correct values for height and pivot angles
  L1(0.375, Math.toRadians(60), .2),
  L2(0, 0, .2),
  L3(0, 0, .2),
  L4(1.4, Math.toRadians(60), .2),

  AlgaeL2(
      1.5, 0, .2), // TO-DO: Determine correct values for height and pivot angles (algae removal)
  AlgaeL3(2.5, 0, .2);

  private final double height;
  private final double pivotAngle;
  private final double pivotOffset;

  /**
   * Constructs a variant of this enum.
   *
   * @param height The desired height in meters.
   * @param pivotAngle The desired pivot angle in radians.
   */
  private ElevatorLevel(double height, double pivotAngle, double pivotOffset) {
    this.height = height;
    this.pivotAngle = pivotAngle;
    this.pivotOffset = pivotOffset;
  }

  /** Returns the desired height in meters. */
  public double getHeight() {
    return height;
  }

  /** Returns the desired pivot angle in radians. */
  public double getPivotAngle() {
    return pivotAngle;
  }

  /** Returns the pivot offset in meters. */
  public double getPivotOffset() {
    return pivotOffset;
  }
}
