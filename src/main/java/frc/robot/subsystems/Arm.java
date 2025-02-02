/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.ArmParameters;

public class Arm extends SubsystemBase {
  private final TalonFX motor;
  private final DutyCycleEncoder absoluteEncoder;

  private final TrapezoidProfile profile;

  private final ArmFeedforward feedForward;
  private final ProfiledPIDController controller;

  private final TrapezoidProfile.State currentAngle = new TrapezoidProfile.State();
  private final TrapezoidProfile.State goalAngle = new TrapezoidProfile.State();
  private boolean enabled;
  private final Timer timer = new Timer();
  private double currentAngleTime = 0;

  /** Creates a new Arm. */
  public Arm(ArmParameters parameters) {
    setName(parameters.toString());

    feedForward = parameters.getArmFeedforward();
    profile = new TrapezoidProfile(parameters.getConstraints());
    controller = parameters.getProfiledPIDController();

    motor = new TalonFX(parameters.getMotorID());
    absoluteEncoder = new DutyCycleEncoder(parameters.getEncoderID());

    MotorOutputConfigs configs = new MotorOutputConfigs();
    configs.NeutralMode = NeutralModeValue.Brake;
    configs.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(configs);
    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
  }

  /** Updates the sensor state. */
  private void updateSensorState() {
    double newAngle = absoluteEncoder.get() * (2 * Math.PI);
    double currentTime = Timer.getFPGATimestamp();
    if (currentAngleTime != 0) {
      currentAngle.velocity = (newAngle - currentAngle.position) / (currentTime - currentAngleTime);
    }
    currentAngle.position = newAngle;
    currentAngleTime = currentTime;
  }

  /** Sets the goal angle and enables periodic control. */
  public void setGoalAngle(double angle) {
    goalAngle.position = angle;
    enabled = true;
    timer.reset();
    timer.start();
  }

  /** Returns whether the coral arm is at goal angle. */
  public boolean atGoalAngle() {
    return controller.atGoal();
  }

  /** Disables periodic control. */
  public void disable() {
    enabled = false;
    timer.stop();
  }

  @Override
  public void periodic() {
    updateSensorState();
    if (enabled) {
      TrapezoidProfile.State desiredState = profile.calculate(timer.get(), currentAngle, goalAngle);
      double feedforward = feedForward.calculate(desiredState.position, desiredState.velocity);
      double feedback = controller.calculate(currentAngle.position, desiredState);
      double voltage = feedforward + feedback;
      motor.setVoltage(voltage);
    }
  }
}
