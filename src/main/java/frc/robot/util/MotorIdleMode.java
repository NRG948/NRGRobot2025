/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** The motor controller behavior when idle. */
public enum MotorIdleMode {
  /**
   * The motor controller effectively disconnects the motor circuit allowing it to spin down at its
   * natural rate.
   */
  COAST,
  /**
   * The motor controller effectively short circuits the motor causing electrical energy to
   * dissipate quickly and bring it to a quick stop.
   */
  BRAKE;

  /** Returns the TalonFX equivalent of this idle mode. */
  public NeutralModeValue forTalonFX() {
    return this == COAST ? NeutralModeValue.Coast : NeutralModeValue.Brake;
  }

  /** Returns the SparkMax/SparkFlex equivalent of this idle mode. */
  public IdleMode forSpark() {
    return this == COAST ? IdleMode.kCoast : IdleMode.kBrake;
  }
}
