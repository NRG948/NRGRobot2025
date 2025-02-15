/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.RobotConstants;

/** A class to hold the feedforward constants calculated from maximum velocity and acceleration. */
public enum ArmParameters {
  // TODO: Update algae arm + coral arm enum values
  CoralArm(
      MotorParameters.KrakenX60,
      1.25,
      3 * 9 * 54 / 36,
      0.315,
      1,
      0.0656,
      RobotConstants.CAN.TalonFX.CORAL_ARM_MOTOR_ID,
      RobotConstants.DigitalIO.CORAL_ARM_ABSOLUTE_ENCODER,
      true, // The abolsute encoder is inverted.
      // The absolute encoder zero point is applied before inversion. When
      // reading the raw value of the absolute encoder to obtain the zero point,
      // we must invert the value by subtracting it from 1 before converting it
      // to radians.
      (1.0 - 0.904) * (2 * Math.PI),
      Math.toRadians(10),
      Math.toRadians(90)),
  AlgaeArm(
      MotorParameters.KrakenX60,
      1,
      1,
      Units.inchesToMeters(20),
      1,
      1,
      RobotConstants.CAN.TalonFX.ALGAE_ARM_MOTOR_ID,
      RobotConstants.DigitalIO.ALGAE_ARM_ABSOLUTE_ENCODER,
      false, // TODO: determine inversion
      0, // TODO: get real encoder offset
      Math.toRadians(45),
      Math.toRadians(90));

  private final MotorParameters motorParameters;
  private final double gearRatio;
  private final double mass;
  private final double armLength;
  private final double efficiency;
  private final int motorID;
  private final int encoderID;
  private final double minAngleRad;
  private final double maxAngleRad;

  private final boolean absoluteEncoderInverted;
  private final double absoluteEncoderZeroOffset;

  private double kS;
  private double kV;
  private double kA;
  private double kG;

  /**
   * Constructs a new ArmParameters.
   *
   * @param motorParameters The motor parameters.
   * @param mass The mass of the arm.
   * @param gearRatio The gear ratio.
   * @param armLength The length of the arm.
   * @param efficiency The efficiency of the arm.
   * @param kS The kS feedforward constant in volts.
   * @param motorID The CAN ID of the motor.
   * @param encoderID The absolute encoder ID.
   * @param absoluteEncoderInverted Whether the absolute encoder is inverted.
   * @param absoluteEncoderZeroOffset The reading of the absolute encoder in radians at the
   *     designated 0 point of the mechanism.
   * @param minAngleRad The min angle of the arm in radians.
   * @param maxAngleRad The max angle of the arm in radians.
   */
  private ArmParameters(
      MotorParameters motorParameters,
      double mass,
      double gearRatio,
      double armLength,
      double efficiency,
      double kS,
      int motorID,
      int encoderID,
      boolean absoluteEncoderInverted,
      double absoluteEncoderZeroOffset,
      double minAngleRad,
      double maxAngleRad) {
    this.gearRatio = gearRatio;
    this.motorParameters = motorParameters;
    this.mass = mass;
    this.armLength = armLength;
    this.efficiency = efficiency;
    this.kS = kS;
    this.motorID = motorID;
    this.encoderID = encoderID;
    this.absoluteEncoderInverted = absoluteEncoderInverted;
    this.absoluteEncoderZeroOffset = absoluteEncoderZeroOffset;
    this.minAngleRad = minAngleRad;
    this.maxAngleRad = maxAngleRad;
    kV = (RobotConstants.MAX_BATTERY_VOLTAGE - kS) / getMaxAngularSpeed();
    kA = (RobotConstants.MAX_BATTERY_VOLTAGE - kS) / getMaxAngularAcceleration();
    kG = kA * 9.81;
  }

  /** Returns the min angle of the arm in radians. */
  public double getMinAngleRad() {
    return minAngleRad;
  }

  /** Returns the max angle of the arm in radians. */
  public double getMaxAngleRad() {
    return maxAngleRad;
  }

  /** Returns the gear ratio. */
  public double getGearRatio() {
    return gearRatio;
  }

  /** Returns the radians per revolution */
  public double getRadiansPerRevolution() {
    return (2 * Math.PI) / gearRatio;
  }

  /** Returns the robot motor parameters. */
  public MotorParameters getMotorParameters() {
    return motorParameters;
  }

  /** Returns the robot mass. */
  public double getMass() {
    return mass;
  }

  /** Returns the robot arm length. */
  public double getArmLength() {
    return armLength;
  }

  /** Returns the efficiency. */
  public double getEfficiency() {
    return efficiency;
  }

  /** Returns the absolute encoder reading in radians at the designated 0 point of the mechanism */
  public double getAbsoluteEncoderZeroOffset() {
    return absoluteEncoderZeroOffset;
  }

  /** Returns whether the absolute encoder is inverted. */
  public boolean isAbsoluteEncoderInverted() {
    return absoluteEncoderInverted;
  }

  /** Returns kS feedforward constant in volts. */
  public double getkS() {
    return kS;
  }

  /** Returns kV feedforward constant in Vs/rad. */
  public double getkV() {
    return kV;
  }

  /** Returns kA feedforward constant Vs^2/rad. */
  public double getkA() {
    return kA;
  }

  /** Returns kG feedforward constant Vs^2/rad. */
  public double getkG() {
    return 9.81 * kA;
  }

  /** Returns the CAN ID of the motor. */
  public int getMotorID() {
    return motorID;
  }

  /** Returns the Encoder ID. */
  public int getEncoderID() {
    return encoderID;
  }

  /** Returns the max angular speed in rad/s. */
  public double getMaxAngularSpeed() {
    return (this.efficiency * this.motorParameters.getDCMotor().freeSpeedRadPerSec)
        / this.gearRatio;
  }

  /** Returns the max angular acceleration in rad/s^2. */
  public double getMaxAngularAcceleration() {
    return (this.efficiency
            * this.motorParameters.getDCMotor().stallTorqueNewtonMeters
            * this.gearRatio)
        / (this.mass * this.armLength);
  }

  /** Returns an {@link ArmFeedforward} object for use with the arm. */
  public ArmFeedforward getArmFeedforward() {
    return new ArmFeedforward(kS, kG, kV, kA);
  }

  /**
   * Returns constraints limiting the maximum angluar velocity and acceleration to 30% and 50% of
   * maximum, respectively.
   */
  public TrapezoidProfile.Constraints getConstraints() {
    return new TrapezoidProfile.Constraints(
        getMaxAngularSpeed() * 0.3, getMaxAngularAcceleration() * 0.5);
  }

  /** Returns a {@link ProfiledPIDController} object for use with the arm. */
  public ProfiledPIDController getProfiledPIDController() {
    return new ProfiledPIDController(1.0, 0, 0, getConstraints());
  }
}
