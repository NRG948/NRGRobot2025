/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.MAX_BATTERY_VOLTAGE;
import static frc.robot.Constants.RobotConstants.PERIODIC_INTERVAL;
import static frc.robot.RobotContainer.RobotSelector.CompetitionRobot2025;
import static frc.robot.RobotContainer.RobotSelector.PracticeRobot2025;
import static frc.robot.parameters.MotorParameters.KrakenX60;

import com.ctre.phoenix6.hardware.TalonFX;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.parameters.ElevatorParameters;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;
import java.util.Map;

@RobotPreferencesLayout(groupName = "Elevator", row = 0, column = 5, width = 1, height = 3)
public class Elevator extends SubsystemBase implements ActiveSubsystem, DataPublisher {

  private static final DataLog LOG = DataLogManager.getLog();

  private static final double GOAL_HEIGHT_TOLERANCE = 0.01; // meters
  private static final double POSITION_ERROR_MARGIN = 0.05; // meters
  private static final double POSITION_ERROR_TIME = 2.0; // seconds

  @RobotPreferencesValue
  public static RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Elevator", "Enable Tab", false);

  public static final ElevatorParameters PARAMETERS =
      RobotContainer.ROBOT_TYPE
          .select(
              Map.of(
                  PracticeRobot2025, ElevatorParameters.PracticeBase2025,
                  CompetitionRobot2025, ElevatorParameters.CompetitionBase2025))
          .orElse(ElevatorParameters.CompetitionBase2025);

  // Physical parameters of the elevator
  public static final double PRACTICE_BOT_GEAR_RATIO = 9.0 / 2;
  public static final double COMPETITION_BOT_GEAR_RATIO = 9.0 / 2;
  private static final double GEAR_RATIO = PARAMETERS.getGearRatio();
  private static final double SPROCKET_DIAMETER = 0.05; // meters
  private static final double MASS = PARAMETERS.getMass(); // kilograms
  private static final double METERS_PER_REVOLUTION = (SPROCKET_DIAMETER * Math.PI) / GEAR_RATIO;

  private static final double MAX_HEIGHT = PARAMETERS.getMaxHeight(); // meters
  private static final double MIN_HEIGHT = PARAMETERS.getMinHeight(); // meters
  private static final double DISABLE_HEIGHT = MIN_HEIGHT + 0.01;
  public static final double STOWED_HEIGHT_FOR_PID = (MIN_HEIGHT + DISABLE_HEIGHT) / 2;

  // Trapezoid profile values
  private static final DCMotor MOTOR_PARAMS = DCMotor.getKrakenX60(1);
  private static final double MAX_SPEED =
      (MOTOR_PARAMS.freeSpeedRadPerSec / (2 * Math.PI)) * METERS_PER_REVOLUTION; // m/s
  private static final double MAX_ACCELERATION =
      (MOTOR_PARAMS.stallTorqueNewtonMeters * GEAR_RATIO) / (SPROCKET_DIAMETER * MASS); // m/s^2
  private static final ExponentialProfile.Constraints EXPONENTIAL_CONSTRAINTS =
      ExponentialProfile.Constraints.fromCharacteristics(
          MAX_BATTERY_VOLTAGE, MAX_SPEED / 2, MAX_ACCELERATION / 64);

  // Feedforward constants
  private static final double KS = KrakenX60.getKs();
  private static final double KV = (MAX_BATTERY_VOLTAGE - KS) / MAX_SPEED;
  private static final double KA = (MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;
  private static final double KG = 9.81 * KA;

  // feedback constants
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KP =
      new RobotPreferences.DoubleValue("Elevator", "KP", 40);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KI =
      new RobotPreferences.DoubleValue("Elevator", "KI", 0);

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue KD =
      new RobotPreferences.DoubleValue("Elevator", "KD", 0);

  private final TalonFXAdapter mainMotor =
      new TalonFXAdapter(
          "/Elevator",
          new TalonFX(PARAMETERS.getMotorID(), "rio"),
          PARAMETERS.getMotorDirection(),
          MotorIdleMode.BRAKE,
          METERS_PER_REVOLUTION);

  private final RelativeEncoder encoder = mainMotor.getEncoder();

  private ElevatorSim simElevator =
      new ElevatorSim(MOTOR_PARAMS, GEAR_RATIO, MASS, SPROCKET_DIAMETER / 2, 0, 1, true, 0);

  private final Mechanism2d mechanism2d = new Mechanism2d(0.5, 1.0);
  private final MechanismRoot2d mechanismRoot2d = mechanism2d.getRoot("Elevator Root", 0, 0);
  private final MechanismLigament2d elevatorMech2d =
      mechanismRoot2d.append(new MechanismLigament2d("Elevator", 0, 90));

  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(KS, KG, KV, KA);
  private final ExponentialProfile profile = new ExponentialProfile(EXPONENTIAL_CONSTRAINTS);
  private final PIDController controller =
      new PIDController(KP.getValue(), KI.getValue(), KD.getValue());
  private final Timer stuckTimer = new Timer();

  private boolean isSeekingGoal;
  private boolean hasError;
  private final ExponentialProfile.State currentState = new ExponentialProfile.State();
  private final ExponentialProfile.State goalState = new ExponentialProfile.State();
  private ExponentialProfile.State lastState = currentState;
  private boolean atUpperLimit;
  private boolean atLowerLimit;
  private double currentVoltage;

  private ElevatorLevel currentElevatorLevel = ElevatorLevel.STOWED;

  /** The offset below the goal height when it is safe to pivot the arm. */
  private double armPivotHeightOffset = 0;

  private Timer resetEncoderTimer = new Timer();

  private BooleanLogEntry logIsSeekingGoal = new BooleanLogEntry(LOG, "/Elevator/isSeekingGoal");
  private DoubleLogEntry logCurrentVelocity = new DoubleLogEntry(LOG, "/Elevator/velocity");
  private DoubleLogEntry logCurrentPosition = new DoubleLogEntry(LOG, "/Elevator/position");
  private DoubleLogEntry logGoalVelocity = new DoubleLogEntry(LOG, "/Elevator/goalVelocity");
  private DoubleLogEntry logGoalPosition = new DoubleLogEntry(LOG, "/Elevator/goalPosition");
  private DoubleLogEntry logCurrentVoltage = new DoubleLogEntry(LOG, "/Elevator/voltage");
  private DoubleLogEntry logFeedForward = new DoubleLogEntry(LOG, "/Elevator/FeedForward");
  private DoubleLogEntry logPIDOutput = new DoubleLogEntry(LOG, "/Elevator/pidOutput");
  private BooleanLogEntry logAtUpperLimit = new BooleanLogEntry(LOG, "/Elevator/atUpperLimit");
  private BooleanLogEntry logAtLowerLimit = new BooleanLogEntry(LOG, "/Elevator/atLowerLimit");
  private BooleanLogEntry logAtGoalHeight = new BooleanLogEntry(LOG, "/Elevator/atGoalHeight");
  private DoubleLogEntry logDesiredPosition = new DoubleLogEntry(LOG, "/Elevator/desiredPosition");
  private DoubleLogEntry logDesiredVelocity = new DoubleLogEntry(LOG, "/Elevator/desiredVelocity");

  /** Creates a new Elevator. */
  public Elevator() {
    updateTelemetry();
    SmartDashboard.putData("Elevator Sim", mechanism2d);
  }

  @Override
  public void disable() {
    mainMotor.disable();
    lastState = new ExponentialProfile.State();
    isSeekingGoal = false;
    logIsSeekingGoal.append(false);
  }

  /** Returns elevator height. */
  public double getHeight() {
    return this.currentState.position;
  }

  /**
   * Sets elevator goal level.
   *
   * @param level the {@link ElevatorLevel} to seek to.
   */
  public void setGoalHeight(ElevatorLevel level) {
    setGoalHeight(level.getElevatorHeight(), level.getArmOffset());
    currentElevatorLevel = level;
  }

  public ElevatorLevel getCurrentElevatorLevel() {
    return currentElevatorLevel;
  }

  /**
   * Sets elevator goal height in meters.
   *
   * @param height the desired height of the elevator in meters.
   * @param armOffset the delta below the desired {@code height} where the arm is allowed to pivot.
   */
  private void setGoalHeight(double height, double armOffset) {
    isSeekingGoal = true;
    goalState.position = height;
    goalState.velocity = 0;
    armPivotHeightOffset = armOffset;
    lastState = currentState;

    controller.setPID(KP.getValue(), KI.getValue(), KD.getValue());
    controller.reset();

    logIsSeekingGoal.append(true);
    logGoalPosition.append(height);
    logGoalVelocity.append(0);
  }

  /** Returns whether the elevator is at goal height. */
  public boolean atGoalHeight() {
    return MathUtil.isNear(goalState.position, getHeight(), GOAL_HEIGHT_TOLERANCE);
  }

  /** Returns whether the elevator is above the height where it is safe to tip the arm outward. */
  public boolean isAboveSafeArmPivotHeight() {
    return currentState.position >= goalState.position - armPivotHeightOffset;
  }

  /** Returns whether the elevator is seeking to the given {@link ElevatorLevel}. */
  public boolean isSeekingLevel(ElevatorLevel level) {
    return goalState.position == level.getElevatorHeight();
  }

  public boolean isSeekingAboveLevel(ElevatorLevel level) {
    return goalState.position > level.getElevatorHeight();
  }

  private void updateTelemetry() {
    if (RobotBase.isReal()) {
      currentState.position = encoder.getPosition();
      currentState.velocity = encoder.getVelocity();
    } else {
      currentState.position = simElevator.getPositionMeters();
      currentState.velocity = simElevator.getVelocityMetersPerSecond();
    }
    currentState.position += MIN_HEIGHT;

    atUpperLimit = currentState.position >= MAX_HEIGHT;
    atLowerLimit = currentState.position <= DISABLE_HEIGHT;

    checkError();

    elevatorMech2d.setLength(currentState.position);
    logCurrentPosition.append(currentState.position);
    logCurrentVelocity.append(currentState.velocity);
    logAtLowerLimit.append(atLowerLimit);
    logAtUpperLimit.append(atUpperLimit);
    logAtGoalHeight.append(atGoalHeight());
    mainMotor.logTelemetry();
  }

  private void checkError() {
    if (MathUtil.isNear(goalState.position, currentState.position, POSITION_ERROR_MARGIN)) {
      stuckTimer.stop();
      stuckTimer.reset();
    } else {
      if (!stuckTimer.isRunning()) {
        stuckTimer.restart();
      }
    }
    hasError = stuckTimer.hasElapsed(POSITION_ERROR_TIME);
  }

  public boolean hasError() {
    return hasError;
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    // Do not ever put the elevator in COAST mode or else it will crash down.
  }

  @Override
  public void periodic() {
    updateTelemetry();
    if (!isSeekingGoal) {
      return;
    }

    if (atLowerLimit
        && goalState.position == STOWED_HEIGHT_FOR_PID
        && PARAMETERS.resetEncoderWhenStowed()) {
      if (!resetEncoderTimer.isRunning()) {
        resetEncoderTimer.restart();
      } else if (resetEncoderTimer.hasElapsed(2.0)) {
        resetEncoderTimer.stop();
        resetEncoderTimer.reset();
        encoder.reset();
        disable();
        return;
      }
    }

    ExponentialProfile.State desiredState =
        profile.calculate(PERIODIC_INTERVAL, lastState, goalState);

    double feedforward = feedForward.calculate(desiredState.velocity);
    double pidOutput = controller.calculate(currentState.position, desiredState.position);

    currentVoltage = feedforward + pidOutput;
    if ((currentVoltage > 0 && atUpperLimit)) {
      currentVoltage = KG;
    }
    if ((currentVoltage < 0 && atLowerLimit)) {
      currentVoltage = 0;
    }

    mainMotor.setVoltage(currentVoltage);

    lastState = desiredState;

    logPIDOutput.append(pidOutput);
    logFeedForward.append(feedforward);
    logDesiredPosition.append(desiredState.position);
    logDesiredVelocity.append(desiredState.velocity);
    logCurrentVoltage.append(currentVoltage);
  }

  @Override
  public void simulationPeriodic() {
    simElevator.setInputVoltage(currentVoltage);
    simElevator.update(0.020);
  }

  public void publishData() {
    SmartDashboard.putNumber("Elevator/Status/Current Height", this.currentState.position);
    SmartDashboard.putNumber("Elevator/Status/Current Velocity", this.currentState.velocity);
    SmartDashboard.putNumber("Elevator/Status/Goal Height", this.goalState.position);
    SmartDashboard.putNumber("Elevator/Status/Goal Velocity", this.goalState.velocity);
    SmartDashboard.putBoolean("Elevator/Status/Is Enabled", this.isSeekingGoal);
    SmartDashboard.putNumber("Elevator/Status/Max Velocity", MAX_SPEED);
    SmartDashboard.putNumber("Elevator/Status/Max Acceleration", MAX_ACCELERATION);
    SmartDashboard.putNumber("Elevator/Control/Current Height", this.currentState.position);
    SmartDashboard.getNumber("Elevator/Control/Elevator Height", 0);
  }
}
