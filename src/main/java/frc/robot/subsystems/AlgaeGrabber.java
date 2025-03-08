/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CAN.TalonFX.ALGAE_GRABBER_MOTOR_ID;
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
import edu.wpi.first.wpilibj.Timer;
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
    groupName = "AlgaeGrabber",
    row = 0,
    column = 6,
    width = 1,
    height = 1,
    type = "Grid Layout",
    gridColumns = 1,
    gridRows = 3)
public class AlgaeGrabber extends SubsystemBase implements ActiveSubsystem, ShuffleboardProducer {

  @RobotPreferencesValue(column = 0, row = 0)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("AlgaeGrabber", "Enable Tab", false);

  private static final DataLog LOG = DataLogManager.getLog();

  private static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
  private static final double GEAR_RATIO = 5.0;

  private static final double METERS_PER_REVOLUTION = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;
  private static final double MAX_VELOCITY =
      (MotorParameters.KrakenX60.getFreeSpeedRPM() * METERS_PER_REVOLUTION) / 60;

  private static final double KS = KrakenX60.getKs();
  private static final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;

  private static final double ERROR_TIME = 3.0;

  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          "/AlgaeGrabber",
          new TalonFX(ALGAE_GRABBER_MOTOR_ID, "rio"),
          CLOCKWISE_POSITIVE,
          BRAKE,
          METERS_PER_REVOLUTION);
  private final RelativeEncoder encoder = motor.getEncoder();

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private final PIDController pidController = new PIDController(1, 0, 0);
  private final Timer outtakeTimer = new Timer();

  private double goalVelocity = 0;
  private double currentVelocity = 0;
  private boolean enabled;
  private boolean hasAlgae = false;
  private boolean hasError = false;

  private DoubleLogEntry logCurrentVelocity =
      new DoubleLogEntry(LOG, "/AlgaeGrabber/currentVelocity");
  private DoubleLogEntry logGoalVelocity = new DoubleLogEntry(LOG, "/AlgaeGrabber/goalVelocity");
  private BooleanLogEntry logHasAlgae = new BooleanLogEntry(LOG, "/AlgaeGrabber/hasAlgae");
  private DoubleLogEntry logFeedForward = new DoubleLogEntry(LOG, "/AlgaeGrabber/feedforward");
  private DoubleLogEntry logFeedBack = new DoubleLogEntry(LOG, "/AlgaeGrabber/feedback");
  private DoubleLogEntry logVoltage = new DoubleLogEntry(LOG, "/AlgaeGrabber/voltage");

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber() {}

  /** Sets the goal velocity in meters per second. */
  public void setGoalVelocity(double velocity) {
    enabled = true;
    goalVelocity = velocity;
    logGoalVelocity.append(goalVelocity);
  }

  /** Updates hasError if stuckTimer exceeds 3 seconds. */
  public void checkError() {
    hasError = outtakeTimer.hasElapsed(ERROR_TIME);
  }

  /** Disables the subsystem. */
  @Override
  public void disable() {
    enabled = false;
    goalVelocity = 0;
    logGoalVelocity.append(0);
    outtakeTimer.stop();
    outtakeTimer.reset();
    motor.stopMotor();
  }

  /** Returns whether we have coral. */
  public boolean hasAlgae() {
    return hasAlgae;
  }

  /** Returns hasError. */
  public boolean hasError() {
    return hasError;
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
      checkError();

      logFeedForward.append(feedforward);
      logFeedBack.append(feedback);
      logVoltage.append(motorVoltage);
    }
  }

  /** Updates and logs the current sensors states. */
  private void updateTelemetry() {
    currentVelocity = encoder.getVelocity();

    logHasAlgae.update(hasAlgae);
    logCurrentVelocity.append(currentVelocity);
    motor.logTelemetry();
  }

  @Override
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab rollerTab = Shuffleboard.getTab("Algae Grabber");
    ShuffleboardLayout statusLayout =
        rollerTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    statusLayout.addBoolean("Enabled", () -> enabled);
    statusLayout.addDouble("Goal Velocity", () -> goalVelocity);
    statusLayout.addDouble("Current Velocity", () -> currentVelocity);
    statusLayout.addBoolean("Has Algae", () -> hasAlgae);
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
