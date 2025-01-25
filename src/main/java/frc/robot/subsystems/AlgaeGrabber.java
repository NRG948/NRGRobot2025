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

public class AlgaeGrabber extends SubsystemBase {

  private final TalonFX motor = new TalonFX(RobotConstants.CAN.TalonFX.ALGAE_GRABBER_MOTOR_ID);
  private double motorSpeed = 0;

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

  public void disable() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    motor.set(motorSpeed);
  }
}
