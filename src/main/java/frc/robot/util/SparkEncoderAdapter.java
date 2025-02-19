/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

/** A relative encoder implementation based on the REV Robotics Spark motor controllers. */
public final class SparkEncoderAdapter implements RelativeEncoder {
  private final com.revrobotics.RelativeEncoder encoder;

  public SparkEncoderAdapter(com.revrobotics.RelativeEncoder encoder) {
    this.encoder = encoder;
  }

  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void reset() {
    encoder.setPosition(0);
  }
}
