/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class RateLimiter {
  private double rateLimit;
  private double prevVal;
  private double prevTime;

  /**
   * Creates a new RateLimiter with the given rate limit and initial value.
   *
   * @param rateLimit The rate-of-change limit in either direction, in units per second. This is
   *     expected to be positive.
   */
  public RateLimiter(double rateLimit) {
    setRateLimit(rateLimit);
    prevVal = 0;
    prevTime = MathSharedStore.getTimestamp();
  }

  /** Sets the rate limit to the given limit. */
  public void setRateLimit(double rate) {
    assert (rateLimit > 0);
    rateLimit = rate;
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate limit.
   */
  public double calculate(double input) {
    double currentTime = MathSharedStore.getTimestamp();
    double elapsedTime = currentTime - prevTime;
    prevTime = currentTime;
    double changeLimit = rateLimit * elapsedTime;
    prevVal += MathUtil.clamp(input - prevVal, -changeLimit, changeLimit);
    return prevVal;
  }

  /**
   * Resets the rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    prevVal = value;
    prevTime = MathSharedStore.getTimestamp();
  }
}
