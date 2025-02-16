/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** A relative encoder implementation based on the CTR Electronics TalonFX motor controller. */
public final class TalonFXEncoderAdapter implements RelativeEncoder {
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final double metersPerRotation;
  private double initPosition;

  public TalonFXEncoderAdapter(TalonFX controller, double metersPerRotation) {
    position = controller.getPosition();
    velocity = controller.getVelocity();
    this.metersPerRotation = metersPerRotation;
    reset();
  }

  @Override
  public double getPosition() {
    return (position.refresh().getValueAsDouble() * metersPerRotation) - initPosition;
  }

  @Override
  public double getVelocity() {
    return velocity.refresh().getValueAsDouble() * metersPerRotation;
  }

  @Override
  public void reset() {
    initPosition = position.refresh().getValueAsDouble() * metersPerRotation;
  }
}
