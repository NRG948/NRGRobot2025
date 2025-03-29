/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CAN.TalonFX.CORAL_GROUND_INTAKE_GRABBER_MOTOR_ID;
import static frc.robot.Constants.RobotConstants.DigitalIO.CORAL_GROUND_INTAKE_BEAM_BREAK;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.parameters.MotorParameters.KrakenX60;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.hardware.TalonFX;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.MotorParameters;
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
public class CoralGroundIntakeGrabber extends SubsystemBase
    implements ActiveSubsystem, ShuffleboardProducer {

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

  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          "/CoralGroundIntakeGrabber",
          new TalonFX(CORAL_GROUND_INTAKE_GRABBER_MOTOR_ID, "rio"),
          CLOCKWISE_POSITIVE,
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
    hasCoral = !beamBreak.get();
    logHasCoral.update(hasCoral);
    logCurrentVelocity.append(currentVelocity);
    motor.logTelemetry();
  }

  @Override
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab rollerTab = Shuffleboard.getTab("Coral Ground Intake Grabber");
    ShuffleboardLayout statusLayout =
        rollerTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    statusLayout.addBoolean("Enabled", () -> enabled);
    statusLayout.addDouble("Goal Velocity", () -> goalVelocity);
    statusLayout.addDouble("Current Velocity", () -> currentVelocity);
    statusLayout.addBoolean("Has Coral", () -> hasCoral);
    statusLayout.add("Max Velocity", MAX_VELOCITY);

    ShuffleboardLayout controlLayout =
        rollerTab.getLayout("Control", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
    GenericEntry intakeSpeed = controlLayout.add("Intake Speed", 0).getEntry();
    controlLayout.add(
        Commands.runOnce(() -> setGoalVelocity(intakeSpeed.getDouble(0)), this).withName("Intake"));
    GenericEntry holdSpeed = controlLayout.add("Hold Speed", 0).getEntry();
    controlLayout.add(
        Commands.runOnce(() -> setGoalVelocity(holdSpeed.getDouble(0)), this).withName("Hold"));
    controlLayout.add(Commands.runOnce(this::disable, this).withName("Disable"));
  }
}
