/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import frc.robot.Constants.RobotConstants;
import java.util.function.DoubleSupplier;

/** A class to hold the feedforward constants calculated from maximum velocity and acceleration. */
public class CalculatedFeedforwardConstants extends FeedforwardConstants {

  /**
   * Constructs an instance of this class.
   *
   * @param kS The constant describing the voltage needed to overcome the motor's static friction.
   * @param maxSpeed Supplies the maximum speed.
   * @param maxAcceleration Supplies the maximum acceleration.
   */
  public CalculatedFeedforwardConstants(
      double kS, DoubleSupplier maxSpeed, DoubleSupplier maxAcceleration) {
    super(
        kS,
        (RobotConstants.MAX_BATTERY_VOLTAGE - kS) / maxSpeed.getAsDouble(),
        (RobotConstants.MAX_BATTERY_VOLTAGE - kS) / maxAcceleration.getAsDouble());
  }
}
