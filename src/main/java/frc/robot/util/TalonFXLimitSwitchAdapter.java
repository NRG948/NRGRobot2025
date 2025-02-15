/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.StatusSignal;

/** A limit switch implementation based on the CTR Electronics TalonFX motor controller. */
public final class TalonFXLimitSwitchAdapter<T extends Enum<T>> implements LimitSwitch {
  private final StatusSignal<T> limitSwitch;
  private final T pressedState;

  public TalonFXLimitSwitchAdapter(StatusSignal<T> limitSwitch, T pressedState) {
    this.limitSwitch = limitSwitch;
    this.pressedState = pressedState;
  }

  @Override
  public boolean isPressed() {
    return limitSwitch.refresh().getValue() == pressedState;
  }
}
