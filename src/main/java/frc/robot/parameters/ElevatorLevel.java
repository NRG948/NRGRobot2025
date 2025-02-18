/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.subsystems.Elevator;

public enum ElevatorLevel {
  // TODO change armOffsets
  STOWED(
      Elevator.MIN_HEIGHT,
      Math.toRadians(90),
      0), // TO-DO: Determine correct values for height and arm angles
  L1(0.375, Math.toRadians(60), .2),
  L2(0.37, Math.toRadians(60), .2),
  L3(0.78, Math.toRadians(60), .2),
  L4(1.4, Math.toRadians(50), .2),

  AlgaeL2(1.5, 0, .2), // TO-DO: Determine correct values for height and arm angles (algae removal)
  AlgaeL3(2.5, 0, .2);

  private final double elevatorHeight;
  private final double armAngle;
  private final double armOffset;

  /**
   * Constructs a variant of this enum.
   *
   * @param height The desired height in meters.
   * @param armAngle The desired arm angle in radians.
   */
  private ElevatorLevel(double height, double armAngle, double armOffset) {
    this.elevatorHeight = height;
    this.armAngle = armAngle;
    this.armOffset = armOffset;
  }

  /** Returns the desired height in meters. */
  public double getElevatorHeight() {
    return elevatorHeight;
  }

  /** Returns the desired arm angle in radians. */
  public double getArmAngle() {
    return armAngle;
  }

  /** Returns the arm offset in meters. */
  public double getArmOffset() {
    return armOffset;
  }
}
