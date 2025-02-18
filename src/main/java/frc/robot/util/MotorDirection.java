/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.signals.InvertedValue;

/** The direction the motor output shaft rotates when positive voltage is applied. */
public enum MotorDirection {
  /** The motor output shaft rotates counter-clockwise when positive voltage is applied. */
  COUNTER_CLOCKWISE_POSITIVE(InvertedValue.CounterClockwise_Positive),

  /** The motor output shaft rotates clockwise when positive voltage is applied. */
  CLOCKWISE_POSITIVE(InvertedValue.Clockwise_Positive);

  private final InvertedValue invertedValue;

  /**
   * Constructs a variant of this enum.
   *
   * @param invertedValue The corresponding {@link InvertedValue} variant for the TalonFX.
   */
  MotorDirection(InvertedValue invertedValue) {
    this.invertedValue = invertedValue;
  }

  /** Returns the corresponding {@link InvertedValue} variant for the TalonFX. */
  public InvertedValue forTalonFX() {
    return invertedValue;
  }

  /**
   * Returns true if the motor rotates in the clockwise direction when a positive voltage is applied
   * (i.e. the motor is "inverted"). Otherwise, this method returns false if the motor rotates in
   * the counter-clockwise direction.
   */
  public boolean isInverted() {
    return this == CLOCKWISE_POSITIVE;
  }
}
