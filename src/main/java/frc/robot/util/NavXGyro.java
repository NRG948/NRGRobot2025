/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;
import edu.wpi.first.math.geometry.Rotation2d;

public class NavXGyro implements Gyro {
  private AHRS ahrs = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k50Hz);

  @Override
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  @Override
  public void reset() {
    ahrs.reset();
  }
}
