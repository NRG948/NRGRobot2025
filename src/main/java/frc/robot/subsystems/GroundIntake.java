/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CAN.TalonFX.GROUND_INTAKE_MOTOR_ID;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.parameters.MotorParameters.KrakenX60;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;

public class GroundIntake extends SubsystemBase implements ActiveSubsystem, ShuffleboardProducer {

  private static final DataLog LOG = DataLogManager.getLog();

  private static final double WHEEL_DIAMETER = 0; // TODO: Find real value
  private static final double GEAR_RATIO = 0; // TODO: Find real value

  private static final double METERS_PER_REVOLUTION = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;
  private static final double MAX_VELOCITY =
      (MotorParameters.KrakenX60.getFreeSpeedRPM() * METERS_PER_REVOLUTION) / 60;

  private static final double KS = KrakenX60.getKs();
  private static final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;

  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          "/GroundIntake",
          new TalonFX(GROUND_INTAKE_MOTOR_ID, "rio"),
          CLOCKWISE_POSITIVE,
          BRAKE,
          METERS_PER_REVOLUTION);
  private final RelativeEncoder encoder = motor.getEncoder();

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private final PIDController pidController = new PIDController(1, 0, 0);

  private double goalVelocity = 0;
  private double currentVelocity = 0;
  private boolean enabled;

  private DoubleLogEntry logCurrentVelocity =
      new DoubleLogEntry(LOG, "/GroundIntake/currentVelocity");
  private DoubleLogEntry logGoalVelocity = new DoubleLogEntry(LOG, "/GroundIntake/goalVelocity");
  private DoubleLogEntry logFeedForward = new DoubleLogEntry(LOG, "/GroundIntake/feedforward");
  private DoubleLogEntry logFeedBack = new DoubleLogEntry(LOG, "/GroundIntake/feedback");
  private DoubleLogEntry logVoltage = new DoubleLogEntry(LOG, "/GroundIntake/voltage");

  /** Creates a new Ground Intake. */
  public GroundIntake() {}

  /** Sets the goal velocity in meters per second. */
  public void setGoalVelocity(double velocity) {
    enabled = true;
    goalVelocity = velocity;
    logGoalVelocity.append(goalVelocity);
  }

  /** Disables the subsystem. */
  @Override
  public void disable() {
    enabled = false;
    goalVelocity = 0;
    logGoalVelocity.append(0);
    motor.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    motor.setIdleMode(idleMode);
  }

  @Override
  public void periodic() {
    updateTelemetry();
    if (enabled) {
      double feedforward = this.feedforward.calculate(goalVelocity);
      double feedback = pidController.calculate(currentVelocity, goalVelocity);
      double motorVoltage = feedforward + feedback;
      motor.setVoltage(motorVoltage);

      logFeedForward.append(feedforward);
      logFeedBack.append(feedback);
      logVoltage.append(motorVoltage);
    }
  }

  /** Updates and logs the current sensors states. */
  private void updateTelemetry() {
    currentVelocity = encoder.getVelocity();
    logCurrentVelocity.append(currentVelocity);
    motor.logTelemetry();
  }

  @Override
  public void addShuffleboardTab() {}
}
