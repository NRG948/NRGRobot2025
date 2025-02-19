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
  private final double distancePerRotation;
  private double initPosition;

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
    position = controller.getPosition();
    velocity = controller.getVelocity();
    this.distancePerRotation = distancePerRotation;
    reset();
  }

  @Override
  public double getPosition() {
    return (position.refresh().getValueAsDouble() * distancePerRotation) - initPosition;
  }

  @Override
  public double getVelocity() {
    return velocity.refresh().getValueAsDouble() * distancePerRotation;
  }

  @Override
  public void reset() {
    initPosition = position.refresh().getValueAsDouble() * distancePerRotation;
  }
}
