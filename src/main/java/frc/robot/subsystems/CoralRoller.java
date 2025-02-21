/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.DigitalIO.CORAL_ROLLER_BEAM_BREAK;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.MotorParameters;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;
import java.util.Set;

@RobotPreferencesLayout(groupName = "CoralRoller", row = 2, column = 0, width = 1, height = 2)
public class CoralRoller extends SubsystemBase implements ActiveSubsystem, ShuffleboardProducer {

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("CoralRoller", "Enable Tab", false);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue INTAKE_VELOCITY =
      new RobotPreferences.DoubleValue("CoralRoller", "Intake Velocity", 1);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue OUTTAKE_VELOCITY =
      new RobotPreferences.DoubleValue("CoralRoller", "Outtake Velocity", 2);

  private static final DataLog LOG = DataLogManager.getLog();

  private static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
  private static final double GEAR_RATIO = 9;

  private static final double METERS_PER_REVOLUTION = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;
  private static final double MAX_VELOCITY =
      (MotorParameters.KrakenX60.getFreeSpeedRPM() * METERS_PER_REVOLUTION) / 60;

  private static final double KS = 0.0656;
  private static final double KV = (RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;

  private static final double ERROR_TIME = 3.0;

  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          new TalonFX(RobotConstants.CAN.TalonFX.CORAL_ROLLER_MOTOR_ID),
          MotorDirection.CLOCKWISE_POSITIVE,
          MotorIdleMode.BRAKE,
          METERS_PER_REVOLUTION);
  private final RelativeEncoder encoder = motor.getEncoder();
  private DigitalInput beamBreak = new DigitalInput(CORAL_ROLLER_BEAM_BREAK);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private final PIDController pidController = new PIDController(1, 0, 0);
  private final Timer outtakeTimer = new Timer();

  private double goalVelocity = 0;
  private double currentVelocity = 0;
  private boolean hasCoral = false;
  private boolean hasError = false;

  private DoubleLogEntry logCurrentVelocity =
      new DoubleLogEntry(LOG, "/CoralRoller/currentVelocity");
  private DoubleLogEntry logGoalVelocity = new DoubleLogEntry(LOG, "/CoralRoller/goalVelocity");
  private BooleanLogEntry logHasCoral = new BooleanLogEntry(LOG, "/CoralRoller/hasCoral");
  private DoubleLogEntry logFeedForward = new DoubleLogEntry(LOG, "/CoralRoller/feedforward");
  private DoubleLogEntry logFeedBack = new DoubleLogEntry(LOG, "/CoralRoller/feedback");
  private DoubleLogEntry logVoltage = new DoubleLogEntry(LOG, "/CoralRoller/voltage");

  /** Creates a new CoralRoller. */
  public CoralRoller() {}

  /** Sets the goal velocity in meters per second. */
  private void setGoalVelocity(double velocity) {
    goalVelocity = velocity;
    logGoalVelocity.append(goalVelocity);
  }

  /** Intakes the coral. */
  public void intake() {
    setGoalVelocity(INTAKE_VELOCITY.getValue());
  }

  /** Outakes the coral. */
  public void outtake() {
    setGoalVelocity(OUTTAKE_VELOCITY.getValue());
    outtakeTimer.restart();
  }

  /** Updates hasError if stuckTimer exceeds 3 seconds. */
  public void checkError() {
    hasError = outtakeTimer.hasElapsed(ERROR_TIME);
  }

  /** Disables the subsystem. */
  @Override
  public void disable() {
    goalVelocity = 0;
    logGoalVelocity.append(0);
    outtakeTimer.stop();
    outtakeTimer.reset();
    motor.stopMotor();
  }

  /** Returns whether we have coral. */
  public boolean hasCoral() {
    return hasCoral;
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
    if (goalVelocity != 0) {
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
    hasCoral = !beamBreak.get();
    currentVelocity = encoder.getVelocity();

    logHasCoral.update(hasCoral);
    logCurrentVelocity.append(currentVelocity);
  }

  @Override
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab rollerTab = Shuffleboard.getTab("Coral Roller");
    ShuffleboardLayout statusLayout =
        rollerTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    statusLayout.addDouble("Goal Velocity", () -> goalVelocity);
    statusLayout.addDouble("Current Velocity", () -> currentVelocity);
    statusLayout.addBoolean("Has Coral", () -> hasCoral);
    statusLayout.add("Max Velocity", MAX_VELOCITY);

    ShuffleboardLayout controlLayout =
        rollerTab.getLayout("Control", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
    GenericEntry speed = controlLayout.add("Speed", 0).getEntry();
    GenericEntry delay = controlLayout.add("Delay", 0).getEntry();
    controlLayout.add(
        Commands.sequence(
                Commands.runOnce(() -> goalVelocity = speed.getDouble(0), this),
                Commands.idle(this).until(this::hasCoral),
                Commands.defer(() -> Commands.waitSeconds(delay.getDouble(0)), Set.of(this)),
                Commands.runOnce(this::disable, this))
            .withName("Intake"));
    controlLayout.add(
        Commands.sequence(
                Commands.runOnce(() -> goalVelocity = speed.getDouble(0), this),
                Commands.idle(this).until(() -> !hasCoral),
                Commands.defer(() -> Commands.waitSeconds(delay.getDouble(0)), Set.of(this)),
                Commands.runOnce(this::disable, this))
            .withName("Deliver"));
    controlLayout.add(Commands.runOnce(this::disable, this).withName("Disable"));
  }
}
