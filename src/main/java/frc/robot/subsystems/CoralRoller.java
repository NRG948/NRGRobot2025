/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CAN.TalonFX.CORAL_ROLLER_MOTOR_ID;
import static frc.robot.Constants.RobotConstants.DigitalIO.CORAL_ROLLER_BEAM_BREAK;
import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.parameters.MotorParameters.KrakenX60;
import static frc.robot.util.MotorDirection.CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
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
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.parameters.MotorParameters;
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
  public static final RobotPreferences.DoubleValue AUTO_CENTER_VELOCITY =
      new RobotPreferences.DoubleValue("CoralRoller", "Auto Center Velocity", -1);

  private static final DataLog LOG = DataLogManager.getLog();

  private static final double WHEEL_DIAMETER = Units.inchesToMeters(3);
  private static final double GEAR_RATIO = 9;

  private static final double METERS_PER_REVOLUTION = (WHEEL_DIAMETER * Math.PI) / GEAR_RATIO;
  private static final double MAX_VELOCITY =
      (MotorParameters.KrakenX60.getFreeSpeedRPM() * METERS_PER_REVOLUTION) / 60;

  private static final double KS = KrakenX60.getKs();
  private static final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_VELOCITY;

  /** A value indicating no measurement was available on the laserCAN distance sensor. */
  private static final double NO_MEASUREMENT = 0.0;

  /** The minimum detection distance from the laserCAN to the reef branch. */
  private static final double REEF_MIN_DISTANCE = Units.inchesToMeters(8.0);

  /** The maximum detection distance from the laserCAN to the reef branch. */
  private static final double REEF_MAX_DISTANCE = REEF_MIN_DISTANCE + Units.inchesToMeters(13.0);

  private static final double ERROR_TIME = 3.0;

  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          "/CoralRoller",
          new TalonFX(CORAL_ROLLER_MOTOR_ID, "rio"),
          CLOCKWISE_POSITIVE,
          BRAKE,
          METERS_PER_REVOLUTION);
  private final RelativeEncoder encoder = motor.getEncoder();
  private DigitalInput beamBreak = new DigitalInput(CORAL_ROLLER_BEAM_BREAK);

  private LaserCan leftLaserCAN = new LaserCan(CAN.LEFT_CORAL_ARM_LASER_CAN_ID);
  private LaserCan rightLaserCAN = new LaserCan(CAN.RIGHT_CORAL_ARM_LASER_CAN_ID);

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(KS, KV);
  private final PIDController pidController = new PIDController(1, 0, 0);
  private final Timer outtakeTimer = new Timer();

  private double leftDistance = NO_MEASUREMENT;
  private double rightDistance = NO_MEASUREMENT;
  private double goalVelocity = 0;
  private double currentVelocity = 0;
  private boolean hasCoral = false;
  private boolean hasError = false;
  private boolean detectsReef = false;
  private boolean leftLaserCANDetected = false;
  private boolean rightLaserCANDetected = false;

  private DoubleLogEntry logLeftDistance = new DoubleLogEntry(LOG, "/CoralRoller/leftDistance");
  private DoubleLogEntry logRightDistance = new DoubleLogEntry(LOG, "/CoralRoller/rightDistance");
  private BooleanLogEntry logDetectsReef = new BooleanLogEntry(LOG, "/CoralRoller/detectsReef");
  private DoubleLogEntry logCurrentVelocity =
      new DoubleLogEntry(LOG, "/CoralRoller/currentVelocity");
  private DoubleLogEntry logGoalVelocity = new DoubleLogEntry(LOG, "/CoralRoller/goalVelocity");
  private BooleanLogEntry logHasCoral = new BooleanLogEntry(LOG, "/CoralRoller/hasCoral");
  private DoubleLogEntry logFeedForward = new DoubleLogEntry(LOG, "/CoralRoller/feedforward");
  private DoubleLogEntry logFeedBack = new DoubleLogEntry(LOG, "/CoralRoller/feedback");
  private DoubleLogEntry logVoltage = new DoubleLogEntry(LOG, "/CoralRoller/voltage");

  /** Creates a new CoralRoller. */
  public CoralRoller() {
    try {
      leftLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      leftLaserCAN.setRegionOfInterest(
          new LaserCan.RegionOfInterest(12, 8, 16, 4)); // Makes detection region a box
      leftLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);

      rightLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      rightLaserCAN.setRegionOfInterest(
          new LaserCan.RegionOfInterest(12, 8, 16, 4)); // Makes detection region a box
      rightLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  /** Sets the goal velocity in meters per second. */
  public void setGoalVelocity(double velocity) {
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

  public boolean detectsReef() {
    return detectsReef;
  }

  public boolean isLeftLaserCANDetected() {
    return leftLaserCANDetected;
  }

  public boolean isRightLaserCANDetected() {
    return rightLaserCANDetected;
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
    } else {
      motor.setVoltage(0);
    }
  }

  /** Updates and logs the current sensors states. */
  private void updateTelemetry() {
    hasCoral = !beamBreak.get();

    Measurement leftMeasurement = leftLaserCAN.getMeasurement();
    Measurement rightMeasurement = rightLaserCAN.getMeasurement();

    if (leftMeasurement != null
        && leftMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      leftDistance = leftMeasurement.distance_mm / 1000.0;
    } else {
      leftDistance = NO_MEASUREMENT;
    }

    if (rightMeasurement != null
        && rightMeasurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      rightDistance = rightMeasurement.distance_mm / 1000.0;
    } else {
      rightDistance = NO_MEASUREMENT;
    }

    leftLaserCANDetected = leftDistance >= REEF_MIN_DISTANCE && leftDistance <= REEF_MAX_DISTANCE;

    rightLaserCANDetected =
        rightDistance >= REEF_MIN_DISTANCE && rightDistance <= REEF_MAX_DISTANCE;

    detectsReef = leftLaserCANDetected && rightLaserCANDetected;

    logLeftDistance.append(leftDistance);
    logRightDistance.append(rightDistance);
    logDetectsReef.append(detectsReef);

    currentVelocity = encoder.getVelocity();
    logHasCoral.update(hasCoral);
    logCurrentVelocity.append(currentVelocity);
    motor.logTelemetry();
  }

  @Override
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab rollerTab = Shuffleboard.getTab("Coral Roller");
    ShuffleboardLayout statusLayout =
        rollerTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 8);
    statusLayout.addDouble("Goal Velocity", () -> goalVelocity);
    statusLayout.addDouble("Current Velocity", () -> currentVelocity);
    statusLayout.addBoolean("Has Coral", () -> hasCoral);
    statusLayout.add("Max Velocity", MAX_VELOCITY);
    statusLayout.addBoolean("Detects Reef", () -> detectsReef);
    statusLayout.addBoolean("Detects Right LaserCAN", () -> rightLaserCANDetected);
    statusLayout.addBoolean("Detects Left LaserCAN", () -> leftLaserCANDetected);
    statusLayout.addDouble("Left Distance", () -> leftDistance);
    statusLayout.addDouble("Right Distance", () -> leftDistance);

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
