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
  private final TalonFX talonFX;
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final double distancePerRotation;

  /**
   * Constructs a TalonFXEncoderAdapter.
   *
   * @param controller The TalonFX object containing the relative encoder to adapt.
   * @param distancePerRotation The distance the attached mechanism moves per rotation of the motor
   *     output shaft.
   *     <p>The unit of measure depends on the mechanism. For a mechanism that produces linear
   *     motion, the unit is typically in meters. For a mechanism that produces rotational motion,
   *     the unit is typically in radians.
   */
  public TalonFXEncoderAdapter(TalonFX controller, double distancePerRotation) {
    talonFX = controller;
    position = controller.getPosition();
    velocity = controller.getVelocity();
    this.distancePerRotation = distancePerRotation;
    reset();
  }

  @Override
  public void setPosition(double position) {
    // The TalonFX encoder position is in units of rotations, so we need to divide by the
    // distance per rotation to get the position in encoder units.
    talonFX.setPosition(position / distancePerRotation);
  }

  @Override
  public double getPosition() {
    // The TalonFX encoder position is in units of rotations, so we need to multiply by the
    // distance per rotation to get the position in the correct units.
    return position.refresh().getValueAsDouble() * distancePerRotation;
  }

  @Override
  public double getVelocity() {
    // The TalonFX encoder velocity is in units of rotations per second, so we need to multiply by
    // the distance per rotation to get the velocity in the correct units.
    return velocity.refresh().getValueAsDouble() * distancePerRotation;
  }

  @Override
  public void reset() {
    talonFX.setPosition(0);
  }
}
