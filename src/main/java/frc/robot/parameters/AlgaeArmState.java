/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

public enum AlgaeArmState {
  STOWED(Math.toRadians(90), 0),
  INTAKE(Math.toRadians(45), 1.5),
  OUTTAKE(Math.toRadians(90), -1.5),
  HOLD(Math.toRadians(90), 0.5),
  BALANCE(Math.toRadians(45), 0);
  private final double armAngle;
  private final double grabberVelocity;

  /**
   * Constructs a variant of this enum.
   *
   * @param armAngle The desired angle of the algae arm in radians.
   * @param grabberVelocity The desired velocity of the algae grabber in meters per second.
   */
  private AlgaeArmState(double armAngle, double grabberVelocity) {
    this.armAngle = armAngle;
    this.grabberVelocity = grabberVelocity;
  }

  /** Returns the angle of the algae arm in radians. */
  public double armAngle() {
    return armAngle;
  }

  /** Returns the velocity of the algae grabber in meters per second. */
  public double grabberVelocity() {
    return grabberVelocity;
  }
}
