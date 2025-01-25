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
      RobotConstants.CAN.TalonFX.CORAL_ARM_MOTOR_ID,
      RobotConstants.DigitalIO.CORAL_ARM_ABSOLUTE_ENCODER),
  AlgaeArm(
      MotorParameters.KrakenX60,
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
  private final int motorID;
  private final int encoderID;

  private double kS;
  private double kV;
  private double kA;
  private double kG;

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
      double kS,
      int motorID,
      int encoderID) {
    this.gearRatio = gearRatio;
    this.motorParameters = motorParameters;
    this.mass = mass;
    this.armLength = armLength;
    this.efficiency = efficiency;
    this.kS = kS;
    this.motorID = motorID;
    this.encoderID = encoderID;
    kV = RobotConstants.MAX_BATTERY_VOLTAGE / getMaxAngularSpeed();
    kA = RobotConstants.MAX_BATTERY_VOLTAGE / getMaxAngularAcceleration();
    kG = kA * 9.81;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public double getRadiansPerRevolution() {
    return (2 * Math.PI) / gearRatio;
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

  public double getkS() {
    return kS;
  }

  public int getMotorID() {
    return motorID;
  }

  public int getEncoderID() {
    return encoderID;
  }

  public double getMaxAngularSpeed() {
    return (this.efficiency * this.motorParameters.getDCMotor().freeSpeedRadPerSec)
        / this.gearRatio;
  }

  public double getMaxAngularAcceleration() {
    return (this.efficiency
            * 2
            * this.motorParameters.getDCMotor().stallTorqueNewtonMeters
            * this.gearRatio)
        / (this.mass * this.armLength);
  }

  public ArmFeedforward getArmFeedforward() {
    return new ArmFeedforward(kS, kG, kV, kA);
  }

  public TrapezoidProfile.Constraints getConstraints() {
    return new TrapezoidProfile.Constraints(
        getMaxAngularSpeed() * 0.3, getMaxAngularAcceleration() * 0.5);
  }

  public ProfiledPIDController getProfiledPIDController() {
    return new ProfiledPIDController(1.0, 0, 0, getConstraints());
  }
}
