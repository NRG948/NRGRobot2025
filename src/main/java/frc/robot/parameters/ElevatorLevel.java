/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public enum ElevatorLevel {
  STOWED(Elevator.STOWED_HEIGHT_FOR_PID, Math.toRadians(92), 0, 0),
  L1(0.15, Math.toRadians(36), .2, 0),
  L2(0.33, Math.toRadians(60), .2, 0),
  L3(0.74, Math.toRadians(60), .2, 0),
  L4(1.34, Math.toRadians(50), .2, .5),

  AlgaeL2(0.25, Math.toRadians(40), .2, 0),
  AlgaeL3(0.65, Math.toRadians(40), .2, 0);

  private final double elevatorHeight;
  private final double armAngle;
  private final double armOffset;
  private double outtakeDelay;

  /**
   * Constructs a variant of this enum.
   *
   * @param height The desired height in meters.
   * @param armAngle The desired arm angle in radians.
   */
  private ElevatorLevel(double height, double armAngle, double armOffset, double outtakeDelay) {
    this.elevatorHeight = height;
    this.armAngle = armAngle;
    this.armOffset = armOffset;
    this.outtakeDelay = outtakeDelay;
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

  /** Returns the outtake speed in radians per second. */
  public double getOuttakeSpeed() {
    switch (this) {
      case L1:
        return 0.6;
      case L4:
        return 2.0;
      case L2:
        return 1.5;
      case L3:
      default:
        return RobotContainer.getOuttakeSpeedL3();
    }
  }

  /** Gets the outtake delay in seconds. */
  public double getOuttakeDelay() {
    return outtakeDelay;
  }
}
