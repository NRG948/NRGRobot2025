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
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.ArmParameters;
import frc.robot.parameters.MotorParameters;

public class Arm extends SubsystemBase {

  private static double MASS; // mass of the arm in Kg
  private static double GEAR_RATIO;
  private static MotorParameters MOTOR;
  private static double ARM_LENGTH;

  private static double EFFICIENCY;
  private static double RADIANS_PER_REVOLUTION;
  private static double MAX_ANGULAR_SPEED;
  private static double MAX_ANGULAR_ACCELERATION;
  private static TrapezoidProfile.Constraints CONSTRAINTS;
  private static double KS;
  private static double KV;
  private static double KA;
  private static double KG;

  private final TalonFX motor;
  private final DutyCycleEncoder absoluteEncoder;

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
  public Arm(ArmParameters PARAMETERS) {

    MASS = PARAMETERS.getMass();
    GEAR_RATIO = PARAMETERS.getGearRatio();
    MOTOR = PARAMETERS.getMotorParameters();

    EFFICIENCY = PARAMETERS.getEfficiency();
    RADIANS_PER_REVOLUTION = PARAMETERS.getRadiansPerRevolution();
    MAX_ANGULAR_SPEED = EFFICIENCY * MOTOR.getFreeSpeedRPM() * RADIANS_PER_REVOLUTION / 60.0;
    ARM_LENGTH = PARAMETERS.getArmLength();
    MAX_ANGULAR_ACCELERATION =
        EFFICIENCY * ((2 * MOTOR.getStallTorque() * GEAR_RATIO) / (MASS * ARM_LENGTH));
    CONSTRAINTS =
        new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED * 0.3, MAX_ANGULAR_ACCELERATION * 0.5);
    KS = PARAMETERS.getkS();
    KV = RobotConstants.MAX_BATTERY_VOLTAGE / MAX_ANGULAR_SPEED;
    KA = RobotConstants.MAX_BATTERY_VOLTAGE / MAX_ANGULAR_ACCELERATION;
    KG = KA * 9.81;

    motor = new TalonFX(PARAMETERS.getMotorID());
    absoluteEncoder = new DutyCycleEncoder(PARAMETERS.getEncoderID());

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
