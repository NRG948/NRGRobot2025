/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.revrobotics.spark.SparkLimitSwitch;

/** A limit switch implementation based on the REV Robotics Spark motor controllers. */
public final class SparkLimitSwitchAdapter implements LimitSwitch {
  private final SparkLimitSwitch limitSwitch;

  public SparkLimitSwitchAdapter(SparkLimitSwitch limitSwitch) {
    this.limitSwitch = limitSwitch;
  }

  @Override
  public boolean isPressed() {
    return limitSwitch.isPressed();
  }
}
