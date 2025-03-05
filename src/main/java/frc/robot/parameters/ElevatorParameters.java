/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.subsystems.Elevator;
import frc.robot.util.MotorDirection;

public enum ElevatorParameters {
  PracticeBase2025(
      3.65,
      Elevator.PRACTICE_BOT_GEAR_RATIO,
      0,
      1.39,
      14,
      false,
      MotorDirection.CLOCKWISE_POSITIVE),
  CompetitionBase2025(
      3.65,
      Elevator.COMPETITION_BOT_GEAR_RATIO,
      0,
      1.39,
      10,
      false,
      MotorDirection.COUNTER_CLOCKWISE_POSITIVE),
  ;

  private final double mass;
  private final double gearRatio;
  private final double minHeight;
  private final double maxHeight;
  private final int motorID;
  private final boolean resetEncoderWhenStowed;
  private final MotorDirection motorDirection;

  ElevatorParameters(
      double mass,
      double gearRatio,
      double minHeight,
      double maxHeight,
      int motorID,
      boolean resetEncoderWhenStowed,
      MotorDirection motorDirection) {
    this.mass = mass;
    this.gearRatio = gearRatio;
    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
    this.motorID = motorID;
    this.resetEncoderWhenStowed = resetEncoderWhenStowed;
    this.motorDirection = motorDirection;
  }

  public double getMass() {
    return mass;
  }

  public double getGearRatio() {
    return gearRatio;
  }

  public double getMinHeight() {
    return minHeight;
  }

  public double getMaxHeight() {
    return maxHeight;
  }

  public int getMotorID() {
    return motorID;
  }

  public boolean resetEncoderWhenStowed() {
    return resetEncoderWhenStowed;
  }

  public MotorDirection getMotorDirection() {
    return motorDirection;
  }
}
