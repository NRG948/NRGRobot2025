/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.subsystems.Elevator;

public enum ElevatorParameters {
  PracticeBase2025(3.7, Elevator.PRACTICE_BOT_GEAR_RATIO, 0.041, 1.42, 14, 8),
  CompetitionBase2025(1, Elevator.COMP_BOT_GEAR_RATIO, 0.033, 1.42, 9, 10);

  private final double mass;
  private final double gearRatio;
  private final double minHeight;
  private final double maxHeight;
  private final int mainDeviceID;
  private final int followerDeviceID;

  ElevatorParameters(
      double mass,
      double gearRatio,
      double minHeight,
      double maxHeight,
      int mainDeviceID,
      int followerDeviceID) {
    this.mass = mass;
    this.gearRatio = gearRatio;
    this.minHeight = minHeight;
    this.maxHeight = maxHeight;
    this.mainDeviceID = mainDeviceID;
    this.followerDeviceID = followerDeviceID;
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

  public int getMainDeviceID() {
    return mainDeviceID;
  }

  public int getFollowerDeviceID() {
    return followerDeviceID;
  }
}
