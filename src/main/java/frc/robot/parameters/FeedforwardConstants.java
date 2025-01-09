/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

/** A class to hold feedforward constants. */
public class FeedforwardConstants {
  public final double kS;
  public final double kV;
  public final double kA;

  /**
   * Constructs an instance of this class.
   *
   * @param kS The constant describing the voltage needed to overcome the motor's static friction.
   * @param kV The constant describing the voltage needed to maintain a constant velocity.
   * @param kA The constant describing the voltage needed to maintain a constant acceleration.
   */
  public FeedforwardConstants(double kS, double kV, double kA) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }
}
