/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class CoralRoller extends SubsystemBase {

  private final TalonFX motor = new TalonFX(RobotConstants.CAN.TalonFX.CORAL_ROLLER_MOTOR_ID);
  private double motorSpeed = 0;

  /** Creates a new AlgaeGrabber. */
  public CoralRoller() {}

  public void intake() {
    motorSpeed = 1.0;
  }

  public void outtake() {
    motorSpeed = -1.0;
  }

  public void disable() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    motor.set(motorSpeed);
  }
}
