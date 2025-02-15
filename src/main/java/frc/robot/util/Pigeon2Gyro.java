/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;

/** A gyro implementation based on the Pigeon 2. */
public final class Pigeon2Gyro implements Gyro {
  private final Pigeon2 pigeon;
  private final StatusSignal<Angle> angle;

  public Pigeon2Gyro(int canID) {
    pigeon = new Pigeon2(canID, "rio");
    angle = pigeon.getYaw();
  }

  @Override
  public double getAngle() {
    return Math.toRadians(angle.refresh().getValueAsDouble());
  }

  @Override
  public void reset() {
    pigeon.reset();
  }
}
