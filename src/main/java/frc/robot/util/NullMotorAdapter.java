/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

public class NullMotorAdapter implements MotorController {

  public NullMotorAdapter() {}

  @Override
  public void set(double speed) {}

  @Override
  public double get() {
    return 0;
  }

  @Override
  public void setInverted(boolean isInverted) {}

  @Override
  public boolean getInverted() {
    return false;
  }

  @Override
  public void disable() {}

  @Override
  public void stopMotor() {}

  @Override
  public MotorController createFollower(int deviceID, boolean isInvertedFromLeader) {
    return new NullMotorAdapter();
  }

  @Override
  public RelativeEncoder getEncoder() {
    return new RelativeEncoder() {

      @Override
      public void setPosition(double position) {}

      @Override
      public double getPosition() {
        return 0;
      }

      @Override
      public double getVelocity() {
        return 0;
      }

      @Override
      public void reset() {}
    };
  }

  @Override
  public LimitSwitch getForwardLimitSwitch() {
    return new LimitSwitch() {

      @Override
      public boolean isPressed() {
        return false;
      }
    };
  }

  @Override
  public LimitSwitch getReverseLimitSwitch() {
    return getForwardLimitSwitch();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {}

  @Override
  public void logTelemetry() {}
}
