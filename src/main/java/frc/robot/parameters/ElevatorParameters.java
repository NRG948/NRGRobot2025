/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.subsystems.Elevator;

public enum ElevatorParameters {
  PracticeBase2025(3.7, Elevator.PRACTICE_BOT_GEAR_RATIO, 0.041, 1.35, 14),
  CompetitionBase2025(3.7, Elevator.COMP_BOT_GEAR_RATIO, 0.041, 1.35, 9);

  private final double mass;
  private final double gearRatio;
  private final double minHeight;
  private final double maxHeight;
  private final int motorID;

  ElevatorParameters(
      double mass, double gearRatio, double minHeight, double maxHeight, int motorID) {
    this.mass = mass;
    this.gearRatio = gearRatio;
    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
    this.motorID = motorID;
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
}
