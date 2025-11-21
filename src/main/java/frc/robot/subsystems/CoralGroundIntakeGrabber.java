/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CAN.TalonFX.NEW_CORAL_GROUND_INTAKE_GRABBER_MOTOR_ID;
import static frc.robot.Constants.RobotConstants.CAN.TalonFX.OLD_CORAL_GROUND_INTAKE_GRABBER_MOTOR_ID;
import static frc.robot.Constants.RobotConstants.DigitalIO.CORAL_GROUND_INTAKE_BEAM_BREAK;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.RobotContainer.RobotSelector.CompetitionRobot2025;
import static frc.robot.RobotContainer.RobotSelector.PracticeRobot2025;
import static frc.robot.parameters.MotorParameters.KrakenX60;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.hardware.TalonFX;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;

@RobotPreferencesLayout(
    groupName = "CoralGroundIntakeGrabber",
    row = 2,
    column = 2,
    width = 2,
    height = 1,
    type = "Grid Layout",
    gridColumns = 1,
    gridRows = 3)
public class CoralGroundIntakeGrabber extends SubsystemBase implements ActiveSubsystem {

  @RobotPreferencesValue(column = 0, row = 0)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("CoralGroundIntakeGrabber", "Enable Tab", false);

  private static final DataLog LOG = DataLogManager.getLog();

  private static final double WHEEL_DIAMETER = Units.inchesToMeters(3.5);
  private static final double GEAR_RATIO = 9.0;

  private static final double METERS_PER_REVOLUTION = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;
  private static final double MAX_VELOCITY =
      (MotorParameters.KrakenX60.getFreeSpeedRPM() * METERS_PER_REVOLUTION) / 60;

  private static final double KS = KrakenX60.getKs();
  private static final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;

  private static final int MOTOR_ID =
      RobotContainer.isCompBot()
          ? OLD_CORAL_GROUND_INTAKE_GRABBER_MOTOR_ID
          : NEW_CORAL_GROUND_INTAKE_GRABBER_MOTOR_ID;
  private static final MotorDirection MOTOR_DIRECTION =
      RobotContainer.isCompBot() ? CLOCKWISE_POSITIVE : COUNTER_CLOCKWISE_POSITIVE;

  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          "/CoralGroundIntakeGrabber",
          new TalonFX(MOTOR_ID, "rio"),
          MOTOR_DIRECTION,
          BRAKE,
          METERS_PER_REVOLUTION);
  private final RelativeEncoder encoder = motor.getEncoder();
  private DigitalInput beamBreak = new DigitalInput(CORAL_GROUND_INTAKE_BEAM_BREAK);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private final PIDController pidController = new PIDController(1, 0, 0);

  private double goalVelocity = 0;
  private double currentVelocity = 0;
  private boolean enabled;
  private boolean hasCoral = false;

  private DoubleLogEntry logCurrentVelocity =
      new DoubleLogEntry(LOG, "/CoralGroundIntakeGrabber/currentVelocity");
  private DoubleLogEntry logGoalVelocity =
      new DoubleLogEntry(LOG, "/CoralGroundIntakeGrabber/goalVelocity");
  private BooleanLogEntry logHasCoral =
      new BooleanLogEntry(LOG, "/CoralGroundIntakeGrabber/hasCoral");
  private DoubleLogEntry logFeedForward =
      new DoubleLogEntry(LOG, "/CoralGroundIntakeGrabber/feedforward");
  private DoubleLogEntry logFeedBack =
      new DoubleLogEntry(LOG, "/CoralGroundIntakeGrabber/feedback");
  private DoubleLogEntry logVoltage = new DoubleLogEntry(LOG, "/CoralGroundIntakeGrabber/voltage");

  /** Creates a new CoralGroundIntakeGrabber. */
  public CoralGroundIntakeGrabber() {}

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

  /** Returns whether we have coral. */
  public boolean hasCoral() {
    return hasCoral;
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    if (CompetitionRobot2025 != null || PracticeRobot2025 != null) {
      motor.setIdleMode(idleMode);
    } else {
      return;
    }
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
    hasCoral = !beamBreak.get();
    logHasCoral.update(hasCoral);
    logCurrentVelocity.append(currentVelocity);
    motor.logTelemetry();
  }
}
