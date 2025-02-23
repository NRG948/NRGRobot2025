/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface ArmParameters {

  public String getArmName();

  /** Returns the min angle of the arm in radians. */
  public double getMinAngleRad();

  /** Returns the max angle of the arm in radians. */
  public double getMaxAngleRad();

  /** Returns the gear ratio. */
  public double getGearRatio();

  /** Returns the radians per revolution */
  public double getRadiansPerRevolution();

  /** Returns the robot motor parameters. */
  public MotorParameters getMotorParameters();

  /** Returns the robot mass. */
  public double getMass();

  /** Returns the robot arm length. */
  public double getArmLength();

  /** Returns the efficiency. */
  public double getEfficiency();

  /** Returns the absolute encoder reading in radians at the designated 0 point of the mechanism */
  public double getAbsoluteEncoderZeroOffset();

  /** Returns whether the absolute encoder is inverted. */
  public boolean isAbsoluteEncoderInverted();

  /** Returns kS feedforward constant in volts. */
  public double getkS();

  /** Returns kV feedforward constant in Vs/rad. */
  public double getkV();

  /** Returns kA feedforward constant Vs^2/rad. */
  public double getkA();

  /** Returns kG feedforward constant Vs^2/rad. */
  public double getkG();

  /** Returns the CAN ID of the motor. */
  public int getMotorID();

  /** Returns the Encoder ID. */
  public int getEncoderID();

  /** Returns the max angular speed in rad/s. */
  public double getMaxAngularSpeed();

  /** Returns the max angular acceleration in rad/s^2. */
  public double getMaxAngularAcceleration();

  /** Returns an {@link ArmFeedforward} object for use with the arm. */
  public ArmFeedforward getArmFeedforward();

  /**
   * Returns constraints limiting the maximum angluar velocity and acceleration to 30% and 50% of
   * maximum, respectively.
   */
  public TrapezoidProfile.Constraints getConstraints();

  /** Returns a {@link ProfiledPIDController} object for use with the arm. */
  public ProfiledPIDController getProfiledPIDController();
}
