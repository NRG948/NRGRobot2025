/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.subsystems.Elevator;

public enum ElevatorLevel {
  // TODO: revert stowed armAngle back to 94.4 for comp bot.
  STOWED(Elevator.STOWED_HEIGHT_FOR_PID, Math.toRadians(101), 0, 0, 0),
  L1(0.15, Math.toRadians(36), .2, 1.5, 0.3),
  L2(0.33, Math.toRadians(60), .2, 0, 1.5),
  L3(0.74, Math.toRadians(60), .2, 0, 1.5),
  L4(1.34, Math.toRadians(50), 0.65, .1, 2.0),

  AlgaeL2(0.25, Math.toRadians(40), .2, 0, 2.0),
  AlgaeL3(0.65, Math.toRadians(40), .2, 0, 2.0);

  private final double elevatorHeight;
  private final double armAngle;
  private final double armOffset;
  private final double outtakeDelay;
  private final double outtakeSpeed;

  /**
   * Constructs a variant of this enum.
   *
   * @param height The desired height in meters.
   * @param armAngle The desired arm angle in radians.
   */
  private ElevatorLevel(
      double height, double armAngle, double armOffset, double outtakeDelay, double outtakeSpeed) {
    this.elevatorHeight = height;
    this.armAngle = armAngle;
    this.armOffset = armOffset;
    this.outtakeDelay = outtakeDelay;
    this.outtakeSpeed = outtakeSpeed;
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

  /** Returns the outtake speed in meters per second. */
  public double getOuttakeSpeed() {
    return outtakeSpeed;
  }

  /** Gets the outtake delay in seconds. */
  public double getOuttakeDelay() {
    return outtakeDelay;
  }
}
