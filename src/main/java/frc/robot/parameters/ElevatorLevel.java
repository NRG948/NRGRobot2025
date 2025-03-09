/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.subsystems.Elevator;

public enum ElevatorLevel {
  STOWED(Elevator.STOWED_HEIGHT_FOR_PID, Math.toRadians(92), 0),
  L1(0.15, Math.toRadians(36), .2),
  L2(0.33, Math.toRadians(60), .2),
  L3(0.74, Math.toRadians(60), .2),
  L4(1.34, Math.toRadians(50), .2),

  AlgaeL2(0.25, Math.toRadians(40), .2),
  AlgaeL3(0.65, Math.toRadians(40), .2);

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
