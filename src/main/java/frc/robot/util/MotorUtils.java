/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorUtils {
  /** Sets brake mode for TalonFX motors */
  public static void setBrakeMode(TalonFX motor, boolean brakeMode) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    motor.getConfigurator().refresh(motorOutputConfigs);
    motorOutputConfigs.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    motor.getConfigurator().apply(motorOutputConfigs);
  }
}
