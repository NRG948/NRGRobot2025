/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import static frc.robot.Constants.RobotConstants.CAN.TalonFX.CORAL_GROUND_INTAKE_ARM_MOTOR_ID;
import static frc.robot.Constants.RobotConstants.CORAL_GROUND_INTAKE_ARM_MASS_KG;
import static frc.robot.Constants.RobotConstants.CORAL_MASS_KG;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;

import edu.wpi.first.math.util.Units;
import frc.robot.util.MotorDirection;

/** A class to hold the feedforward constants calculated from maximum velocity and acceleration. */
public enum CoralGroundIntakeArmParameters implements ArmParameters {
  PracticeBase2025(
      MotorParameters.KrakenX60,
      CORAL_GROUND_INTAKE_ARM_MASS_KG,
      36.0 * 22.0 / 15.0,
      Units.inchesToMeters(21),
      CORAL_GROUND_INTAKE_ARM_MOTOR_ID,
      Math.toRadians(128),
      Math.toRadians(-35),
      Math.toRadians(128),
      0.15),

  CompetitionBase2025(
      MotorParameters.KrakenX60,
      CORAL_GROUND_INTAKE_ARM_MASS_KG,
      36.0 * 22.0 / 15.0,
      Units.inchesToMeters(21),
      CORAL_GROUND_INTAKE_ARM_MOTOR_ID,
      Math.toRadians(127),
      Math.toRadians(-25),
      Math.toRadians(127),
      0.15);

  private final MotorParameters motorParameters;
  private final double gearRatio;
  private final double mass;
  private final double armLength;
  private final int motorID;
  private final double stowedAngle;
  private final double minAngleRad;
  private final double maxAngleRad;

  private final double kS;
  private final double kV;
  private final double kA;
  private final double kAWithCoral;

  private final double rollerDelay;
  private static final double ACCELERATION_SCALE_FACTOR = 0.05;
  private static final double SPEED_SCALE_FACTOR = 0.7;

  /**
   * Constructs a new ArmParameters.
   *
   * @param motorParameters The motor parameters.
   * @param mass The mass of the arm.
   * @param gearRatio The gear ratio.
   * @param armLength The length of the arm.
   * @param motorID The CAN ID of the motor.
   * @param stowedAngle The angle of the arm when stowed in radians.
   * @param minAngleRad The min angle of the arm in radians.
   * @param maxAngleRad The max angle of the arm in radians.
   * @param rollerDelay The delay delay in seconds from detection of the coral to stopping the motor
   *     during intake.
   */
  private CoralGroundIntakeArmParameters(
      MotorParameters motorParameters,
      double mass,
      double gearRatio,
      double armLength,
      int motorID,
      double stowedAngle,
      double minAngleRad,
      double maxAngleRad,
      double rollerDelay) {
    this.gearRatio = gearRatio;
    this.motorParameters = motorParameters;
    this.mass = mass;
    this.armLength = armLength;
    this.kS = motorParameters.getKs();
    this.motorID = motorID;
    this.stowedAngle = stowedAngle;
    this.minAngleRad = minAngleRad;
    this.maxAngleRad = maxAngleRad;
    this.rollerDelay = rollerDelay;
    kV = (MAX_BATTERY_VOLTAGE - kS) / getMaxAngularSpeed();
    kA = (MAX_BATTERY_VOLTAGE - kS) / getMaxAngularAcceleration();
    kAWithCoral = (MAX_BATTERY_VOLTAGE - kS) / getMaxAngularAccelerationWithCoral();
  }

  /** Returns the name of the arm subsystem. */
  public String getArmName() {
    return "CoralGroundIntakeArm";
  }

  /** Returns the angle of the arm when stowed in radians. */
  public double getStowedAngleRad() {
    return stowedAngle;
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

  /** Returns the direction the motor rotates when a positive voltage is applied. */
  public MotorDirection getMotorDirection() {
    return COUNTER_CLOCKWISE_POSITIVE;
  }

  /** Returns the robot mass. */
  public double getMass() {
    return mass;
  }

  /** Returns the robot arm length. */
  public double getArmLength() {
    return armLength;
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
  public double getkAWithoutCoral() {
    return kA;
  }

  /** Returns kAWithCoral feedforward constant Vs^2/rad. */
  public double getkAWithCoral() {
    return kAWithCoral;
  }

  /** Returns kG feedforward constant Vs^2/rad. */
  public double getkGWithoutCoral() {
    return 9.81 * kA;
  }

  /** Returns kGWithCoral feedforward constant Vs^2/rad. */
  public double getkGWithCoral() {
    return 9.81 * kAWithCoral;
  }

  /** Returns the CAN ID of the motor. */
  public int getMotorID() {
    return motorID;
  }

  /** Returns the max angular speed in rad/s. */
  public double getMaxAngularSpeed() {
    return (this.motorParameters.getDCMotor().freeSpeedRadPerSec) / this.gearRatio;
  }

  public double getAngularSpeed() {
    return this.getMaxAngularSpeed() * SPEED_SCALE_FACTOR;
  }

  /** Returns the max angular acceleration in rad/s^2. */
  public double getMaxAngularAcceleration() {
    return (this.motorParameters.getDCMotor().stallTorqueNewtonMeters * this.gearRatio)
        / (this.mass * this.armLength);
  }

  public double getAngularAcceleration() {
    return this.getMaxAngularAcceleration() * ACCELERATION_SCALE_FACTOR;
  }

  /** Returns the max angular acceleration with coral in rad/s^2. */
  public double getMaxAngularAccelerationWithCoral() {
    return (this.motorParameters.getDCMotor().stallTorqueNewtonMeters * this.gearRatio)
        / ((this.mass + CORAL_MASS_KG) * this.armLength);
  }

  public double getAngularAccelerationWithCoral() {
    return this.getAngularAccelerationWithCoral() * ACCELERATION_SCALE_FACTOR;
  }

  /**
   * Returns the delay in seconds from detection of the coral to stopping the motor during intake.
   */
  public double getRollerDelay() {
    return rollerDelay;
  }
}
