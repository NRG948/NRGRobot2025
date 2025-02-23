/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import static frc.robot.Constants.RobotConstants.WHEEL_DIAMETER;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;

import frc.robot.util.MotorDirection;

/**
 * A enum representing the properties on a specific swerve drive module.
 *
 * <p>The parameters for the MK4 modules are taken from <a
 * href="https://www.swervedrivespecialties.com/products/mk4-swerve-module">MK4 Swerve Module</a>.
 *
 * <p>The parameters for the MK4i modules are taken from <a
 * href="https://www.swervedrivespecialties.com/products/mk4i-swerve-module">MK4i Swerve Module</a>
 * and <a
 * href="https://www.swervedrivespecialties.com/collections/mk4i-parts/products/kit-adapter-16t-drive-pinion-gear-mk4i">Kit,
 * Adapter, 16T Drive Pinion Gear (MK4i)</a>.
 *
 * <p>The calculations for the theoretical maximum speeds and acceleration are taken from the <a
 * href=
 * "https://www.chiefdelphi.com/uploads/default/original/3X/f/7/f79d24101e6f1487e76099774e4ba60683e86cda.pdf">
 * FRC Drivetrain Characterization</a> paper by Noah Gleason and Eli Barnett of FRC Team 449 - The
 * Blair Robot Project.
 */
public enum SwerveModuleParameters {

  /** An MK4 Swerve Module in the L1 configuration. */
  MK4_L1(WHEEL_DIAMETER, 8.14, 12.8, COUNTER_CLOCKWISE_POSITIVE),

  /** An MK4 Swerve Module in the L2 configuration. */
  MK4_L2(WHEEL_DIAMETER, 6.75, 12.8, COUNTER_CLOCKWISE_POSITIVE),

  /** An MK4 Swerve Module in the L3 configuration. */
  MK4_L3(WHEEL_DIAMETER, 6.12, 12.8, COUNTER_CLOCKWISE_POSITIVE),

  /** An MK4 Swerve Module in the L4 configuration. */
  MK4_L4(WHEEL_DIAMETER, 5.14, 12.8, COUNTER_CLOCKWISE_POSITIVE),

  /** An MK4I Swerve Module in the L1 configuration. */
  MK4I_L1(WHEEL_DIAMETER, 8.14, 150.0 / 7.0, CLOCKWISE_POSITIVE),

  /** An MK4I Swerve Module in the L2 configuration. */
  MK4I_L2(WHEEL_DIAMETER, 6.75, 150.0 / 7.0, CLOCKWISE_POSITIVE),

  /** An MK4I Swerve Module in the L2+ configuration. */
  MK4I_L2_PLUS(WHEEL_DIAMETER, 5.9, 150.0 / 7.0, CLOCKWISE_POSITIVE),

  /** An MK4I Swerve Module in the L3 configuration. */
  MK4I_L3(WHEEL_DIAMETER, 6.12, 150.0 / 7.0, CLOCKWISE_POSITIVE);

  /** */
  private final double wheelDiameter;

  private final double driveGearRatio;
  private final double steeringGearRatio;
  private final MotorDirection steeringDirection;

  /**
   * Constructs an instance of this enum.
   *
   * @param wheelDiameter The wheel diameter in meters.
   * @param driveGearRatio The drive gear ratio.
   * @param steeringGearRatio The steering gear ratio.
   * @param steeringDirection The direction the steering motor must rotate when a positive voltage
   *     is applied to rotate the wheel in the counter-clockwise direction.
   */
  SwerveModuleParameters(
      double wheelDiameter,
      double driveGearRatio,
      double steeringGearRatio,
      MotorDirection steeringDirection) {
    this.wheelDiameter = wheelDiameter;
    this.driveGearRatio = driveGearRatio;
    this.steeringGearRatio = steeringGearRatio;
    this.steeringDirection = steeringDirection;
  }

  /**
   * Returns the wheel radius in meters.
   *
   * @return The wheel radius in meters.
   */
  public double getWheelDiameter() {
    return this.wheelDiameter;
  }

  /**
   * Returns the drive gear ratio.
   *
   * @return The drive gear ratio.
   */
  public double getDriveGearRatio() {
    return this.driveGearRatio;
  }

  /**
   * Returns the steering gear ratio.
   *
   * @return The steering gear ratio.
   */
  public double getSteeringGearRatio() {
    return this.steeringGearRatio;
  }

  /**
   * Returns the direction the steering motor must rotate when a positive voltage is applied to
   * rotate the wheel in the counter-clockwise direction.
   *
   * @return The direction the steering motor must rotate when a positive voltage is applied to
   *     rotate the wheel in the counter-clockwise direction.
   */
  public MotorDirection getSteeringDirection() {
    return this.steeringDirection;
  }

  /**
   * Returns the theoretical maximum drive speed in m/s when using the specified motor.
   *
   * @param motor The motor parameters.
   * @return The theoretical maximum drive speed.
   */
  public double calculateMaxDriveSpeed(MotorParameters motor) {
    return (motor.getFreeSpeedRPM() * this.wheelDiameter * Math.PI) / (60.0 * this.driveGearRatio);
  }

  /**
   * Returns the theoretical maximum drive acceleration in m/s^2 when using the specified motor.
   *
   * @param motor The motor parameters.
   * @param robotMass the total robot mass in Kg including bumpers and battery.
   * @return The theoretical maximum drive acceleration.
   */
  public double calculateMaxDriveAcceleration(MotorParameters motor, double robotMass) {
    return (2 * 4 * motor.getStallTorque() * this.driveGearRatio)
        / (this.wheelDiameter * robotMass);
  }

  /**
   * Returns the theoretical maximum steering speed in rad/s when using the specified motor.
   *
   * @param motor The motor parameters.
   * @return The theoretical maximum drive speed.
   */
  public double calculateMaxSteeringSpeed(MotorParameters motor) {
    return (motor.getFreeSpeedRPM() * 2 * Math.PI) / (60.0 * this.steeringGearRatio);
  }

  /**
   * Returns the theoretical maximum drive acceleration in rad/s^2 when using the specified motor.
   *
   * @param motor The motor parameters.
   * @param robotMass the total robot mass in Kg including bumpers and battery.
   * @return The theoretical maximum drive acceleration.
   */
  public double calculateMaxSteeringAcceleration(MotorParameters motor, double robotMass) {
    return (2 * 4 * motor.getStallTorque() * this.steeringGearRatio * 2 * Math.PI) / robotMass;
  }
}
