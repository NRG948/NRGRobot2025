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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.parameters.MotorParameters;

public class CoralArm extends SubsystemBase {

  private static final double MASS = 1.361; // mass of the arm in Kg
  private static final double GEAR_RATIO = 75.0;
  private static final MotorParameters MOTOR = MotorParameters.KrakenX60;

  private static final double EFFICIENCY = 1.0;
  private static final double RADIANS_PER_REVOLUTION = 1.0;
  private static final double MAX_ANGULAR_SPEED =
      EFFICIENCY * MOTOR.getFreeSpeedRPM() * RADIANS_PER_REVOLUTION / 60.0;
  private static final double ARM_LENGTH = Units.inchesToMeters(12);
  private static final double MAX_ANGULAR_ACCELERATION =
      EFFICIENCY * ((2 * MOTOR.getStallTorque() * GEAR_RATIO) / (MASS * ARM_LENGTH));
  private static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.3, MAX_ANGULAR_ACCELERATION * 0.5);
  private static final double KS = 0.15;
  private static final double KV = RobotConstants.MAX_BATTERY_VOLTAGE / MAX_ANGULAR_SPEED;
  private static final double KA = RobotConstants.MAX_BATTERY_VOLTAGE / MAX_ANGULAR_ACCELERATION;
  private static final double KG = KA * 9.81;

  private final TalonFX motor = new TalonFX(RobotConstants.CAN.TalonFX.ARM_MOTOR_ID);
  private final DutyCycleEncoder absoluteEncoder =
      new DutyCycleEncoder(RobotConstants.DigitalIO.ARM_ABSOLUTE_ENCODER);

  private final TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS);

  private final ArmFeedforward feedForward = new ArmFeedforward(KS, KG, KV, KA);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(1.0, 0, 0, CONSTRAINTS);

  private final TrapezoidProfile.State currentAngle = new TrapezoidProfile.State();
  private final TrapezoidProfile.State goalAngle = new TrapezoidProfile.State();
  private boolean enabled;
  private final Timer timer = new Timer();
  private double currentAngleTime = 0;

  /** Creates a new Arm. */
  public CoralArm() {
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
  public void setGoalAngle(ElevatorLevel level) {
    goalAngle.position = level.getPivotAngle();
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
