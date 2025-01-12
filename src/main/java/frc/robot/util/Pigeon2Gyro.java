/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class Pigeon2Gyro implements Gyro {
  private Pigeon2 pigeon;

  public Pigeon2Gyro(int canID) {
    pigeon = new Pigeon2(canID, "rio");
  }

  @Override
  public Rotation2d getAngle() {
    return pigeon.getRotation2d();
  }

  @Override
  public void reset() {
    pigeon.reset();
  }
}
