/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class CoralRoller extends SubsystemBase {

  private final TalonFX motor = new TalonFX(RobotConstants.CAN.TalonFX.CORAL_ROLLER_MOTOR_ID);
  private StatusSignal<ForwardLimitValue> beamBreak = motor.getForwardLimit();

  private double motorSpeed = 0;
  private boolean hasCoral = false;

  /** Creates a new AlgaeGrabber. */
  public CoralRoller() {}

  public void intake() {
    motorSpeed = 0.8; // TODO: test & determine safe maximum speed
  }

  public void outtake() {
    motorSpeed = -0.8;
  }

  public void disable() {
    motor.stopMotor();
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void periodic() {
    motor.set(motorSpeed);
    hasCoral = beamBreak.refresh().getValue() == ForwardLimitValue.Open;
  }
}
