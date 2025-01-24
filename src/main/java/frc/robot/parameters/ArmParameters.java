/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.Constants.RobotConstants;

/** A class to hold the feedforward constants calculated from maximum velocity and acceleration. */
public enum ArmParameters {
  // TODO: Update algae arm + coral arm enum values
  CoralArm(
      MotorParameters.KrakenX60,
      1,
      1,
      1,
      1,
      1,
      1,
      RobotConstants.CAN.TalonFX.CORAL_ARM_MOTOR_ID,
      RobotConstants.DigitalIO.CORAL_ARM_ABSOLUTE_ENCODER),
  AlgaeArm(
      MotorParameters.KrakenX60,
      1,
      1,
      1,
      1,
      1,
      1,
      RobotConstants.CAN.TalonFX.ALGAE_ARM_MOTOR_ID,
      RobotConstants.DigitalIO.ALGAE_ARM_ABSOLUTE_ENCODER);

  private final MotorParameters motorParameters;
  private final double gearRatio;
  private final double mass;
  private final double armLength;
  private final double efficiency;
  private final double radiansPerRevolution;
  private final double kS;
  private final int motorID;
  private final int encoderID;

  /**
   * removed:
   *
   * <p>private final double stowedAngle; private final double rawAngleOffset; private final double
   * CGAngleOffset;
   */
  private ArmParameters(
      MotorParameters motorParameters,
      double mass,
      double gearRatio,
      double armLength,
      double efficiency,
      double radiansPerRevolution,
      double kS,
      int motorID,
      int encoderID) {
    this.gearRatio = gearRatio;
    this.motorParameters = motorParameters;
    this.mass = mass;
    this.armLength = armLength;
    this.efficiency = efficiency;
    this.radiansPerRevolution = radiansPerRevolution;
    this.kS = kS;
    this.motorID = motorID;
    this.encoderID = encoderID;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public MotorParameters getMotorParameters() {
    return motorParameters;
  }

  public double getMass() {
    return mass;
  }

  public double getArmLength() {
    return armLength;
  }

  public double getEfficiency() {
    return efficiency;
  }

  public double getRadiansPerRevolution() {
    return radiansPerRevolution;
  }

  public double getkS() {
    return kS;
  }

  public int getMotorID() {
    return motorID;
  }

  public int getEncoderID() {
    return encoderID;
  }
}
