/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

/**
 * An adapter for the <a href="https://www.revrobotics.com/rev-11-1271/">Rev Throughbore Encoder</a>
 * to implement the AbsoluteAngleEncoder interface.
 */
public class RevThroughboreEncoderAdapter implements AbsoluteAngleEncoder {
  private static final double DUTY_CYCLE_PERIOD = 1025.0;
  private static final double MIN_DUTY_CYCLE = 1.0 / DUTY_CYCLE_PERIOD;
  private static final double MAX_DUTY_CYCLE = 1024.0 / 1025.0;

  private static final double RANGE = 2 * Math.PI;

  private final DutyCycleEncoder encoder;

  /**
   * Creates a new RevThroughboreEncoderAdapter.
   *
   * @param channel The digital input channel for the encoder.
   * @param inverted Whether the encoder is inverted.
   * @param zeroPoint The zero point of the encoder in radians with range -π to π radians.
   *     <p>To get this value, set it to zero initially and read the value from the encoder. Pass
   *     the observed value here.
   */
  public RevThroughboreEncoderAdapter(int channel, boolean inverted, double zeroPoint) {
    // The DutyCycleEncoder outputs a value between 0 and 2*π radians, so we need to adjust the
    // zero point to be in that range.
    zeroPoint = MathUtil.inputModulus(zeroPoint, 0, 2 * Math.PI);

    if (inverted) {
      // The DutyCycleEncoder implementation applies inversion after adjusting the zero point,
      // so we need to subtract the specified zero point from the maximum range value to
      // get the correct value.
      zeroPoint = RANGE - zeroPoint;
    }

    this.encoder = new DutyCycleEncoder(channel, RANGE, zeroPoint);
    this.encoder.setDutyCycleRange(MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
    this.encoder.setInverted(inverted);
  }

  @Override
  public double getAngle() {
    return MathUtil.angleModulus(encoder.get());
  }
}
