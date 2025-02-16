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
import frc.robot.util.MotorIdleMode;
import frc.robot.util.MotorUtils;

public class AlgaeGrabber extends SubsystemBase implements ActiveSubsystem {

  private final TalonFX motor = new TalonFX(RobotConstants.CAN.TalonFX.ALGAE_GRABBER_MOTOR_ID);
  private double motorSpeed = 0;

  private StatusSignal<ForwardLimitValue> beamBreak = motor.getForwardLimit();
  private boolean hasAlgae;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber() {}

  public void intake() {
    motorSpeed = 0.8; // TODO: test & determine safe maximum speed
  }

  public void hold() {
    motorSpeed =
        0.2; // TODO: test & determine motor power needed to hang on to game piece while moving
  }

  public void outtake() {
    motorSpeed = -0.8;
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }

  @Override
  public void disable() {
    motor.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    MotorUtils.setIdleMode(motor, idleMode);
  }

  @Override
  public void periodic() {
    hasAlgae = beamBreak.refresh().getValue() == ForwardLimitValue.Open;
    motor.set(motorSpeed);
  }
}
