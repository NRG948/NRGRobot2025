/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.ElevatorCommands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.parameters.ElevatorParameters;
import frc.robot.util.MotorController;
import frc.robot.util.MotorDirection;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;
import java.util.Set;

@RobotPreferencesLayout(groupName = "Elevator", row = 0, column = 5, width = 2, height = 3)
public class Elevator extends SubsystemBase implements ActiveSubsystem, ShuffleboardProducer {
  private static final double GOAL_POSITION_TOLERANCE = 0.01;
  private static final double POSITION_ERROR_MARGIN = 0.05; // meters
  private static final double POSITION_ERROR_TIME = 2.0;
  private static final double STOWED_POSITION_TOLERANCE = 0.001;

  private static final DataLog LOG = DataLogManager.getLog();

  @RobotPreferencesValue
  public static RobotPreferences.EnumValue<ElevatorParameters> PARAMETERS =
      new RobotPreferences.EnumValue<ElevatorParameters>(
          "Elevator", "Robot Base", ElevatorParameters.CompetitionBase2025);

  @RobotPreferencesValue
  public static RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Elevator", "Enable Tab", false);

  // physical parameters of the elevator
  private static final double GEAR_RATIO = ((60.0 / 12.0) * (15.0 / 15.0)) / 2;
  private static final double SPROCKET_DIAMETER = 0.0474; // in meters
  private static final double MASS = PARAMETERS.getValue().getMass(); // kilograms
  private static final double METERS_PER_REVOLUTION = (SPROCKET_DIAMETER * Math.PI) / GEAR_RATIO;

  private static final double MAX_HEIGHT = PARAMETERS.getValue().getMaxHeight();
  private static final double MIN_HEIGHT = PARAMETERS.getValue().getMinHeight(); // in meters
  private static final double DISABLE_HEIGHT = MIN_HEIGHT + 0.08;
  public static final double STOWED_HEIGHT_FOR_PID = (MIN_HEIGHT + DISABLE_HEIGHT) / 2;
  private static final double COLLISION_VELOCITY_THRESHOLD = 0.001; // in m/s
  private static final double COLLISION_DURATION = 0.25;

  // trapezoid profile values
  private static final DCMotor MOTOR_PARAMS = DCMotor.getKrakenX60(1);
  private static final double MAX_SPEED =
      (MOTOR_PARAMS.freeSpeedRadPerSec / (2 * Math.PI)) * METERS_PER_REVOLUTION; // m/s
  private static final double MAX_ACCELERATION =
      (2 * MOTOR_PARAMS.stallTorqueNewtonMeters * GEAR_RATIO)
          / (SPROCKET_DIAMETER * MASS); // m/s^2 for two motors
  private static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCELERATION / 64);

  // feedforward constants
  /*
   * The KS is calculated from the internal resistance * free speed current.
   * We can calculate the internal resistance of the motor with battery voltage / stall current.
   * R = 12V / 366A, KS = R * 2A for the KrakenX60.
   */
  private static final double KS = 0.0656;
  private static final double KV = (Constants.RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_SPEED;
  private static final double KA =
      (Constants.RobotConstants.MAX_BATTERY_VOLTAGE - KS) / MAX_ACCELERATION;
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
      new RobotPreferences.DoubleValue("Elevator", "KD", 0.5);

  private TalonFXAdapter mainMotor =
      new TalonFXAdapter(
          new TalonFX(PARAMETERS.getValue().getMainDeviceID(), "rio"),
          MotorDirection.COUNTER_CLOCKWISE_POSITIVE,
          MotorIdleMode.BRAKE,
          METERS_PER_REVOLUTION);

  private MotorController follower =
      mainMotor.createFollower(PARAMETERS.getValue().getFollowerDeviceID(), false);

  private RelativeEncoder encoder = mainMotor.getEncoder();

  private ElevatorSim simElevator =
      new ElevatorSim(MOTOR_PARAMS, GEAR_RATIO, MASS, SPROCKET_DIAMETER / 2, 0, 1, true, 0);

  private Mechanism2d mechanism2d = new Mechanism2d(0.5, 1.0);
  private MechanismRoot2d mechanismRoot2d = mechanism2d.getRoot("Elevator Root", 0, 0);
  private MechanismLigament2d elevatorMech2d =
      mechanismRoot2d.append(new MechanismLigament2d("Elevator", 0, 90));

  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(KS, KG, KV, KA);
  private final TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(KP.getValue(), KI.getValue(), KD.getValue(), CONSTRAINTS);
  private final Timer stuckTimer = new Timer();

  private boolean isSeekingGoal;
  private boolean hasError;
  private final TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  private final TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private TrapezoidProfile.State lastState = currentState;
  private boolean atUpperLimit;
  private boolean atLowerLimit;
  private double currentVoltage;
  private final Timer collisionTimer = new Timer();

  /** armOffset starts from the goal height down. */
  private double armOffset = 0;

  private BooleanLogEntry logIsSeekingGoal = new BooleanLogEntry(LOG, "Elevator/isSeekingGoal");
  private DoubleLogEntry logCurrentVelocity = new DoubleLogEntry(LOG, "Elevator/velocity");
  private DoubleLogEntry logCurrentPosition = new DoubleLogEntry(LOG, "Elevator/position");
  private DoubleLogEntry logGoalVelocity = new DoubleLogEntry(LOG, "Elevator/goalVelocity");
  private DoubleLogEntry logGoalPosition = new DoubleLogEntry(LOG, "Elevator/goalPosition");
  private DoubleLogEntry logCurrentVoltage = new DoubleLogEntry(LOG, "Elevator/voltage");
  private DoubleLogEntry logStatorCurrent = new DoubleLogEntry(LOG, "Elevator/statorCurrent");
  private DoubleLogEntry logTorqueCurrent = new DoubleLogEntry(LOG, "Elevator/torqueCurrent");
  private DoubleLogEntry logFeedForward = new DoubleLogEntry(LOG, "Elevator/FeedForward");
  private DoubleLogEntry logPIDOutput = new DoubleLogEntry(LOG, "Elevator/pidOutput");
  private BooleanLogEntry logAtUpperLimit = new BooleanLogEntry(LOG, "Elevator/atUpperLimit");
  private BooleanLogEntry logAtLowerLimit = new BooleanLogEntry(LOG, "Elevator/atLowerLimit");
  private DoubleLogEntry logDesiredPosition = new DoubleLogEntry(LOG, "Elevator/desiredPosition");
  private DoubleLogEntry logDesiredVelocity = new DoubleLogEntry(LOG, "Elevator/desiredVelocity");

  /** Creates a new Elevator. */
  public Elevator() {
    updateSensorState();
    SmartDashboard.putData("Elevator Sim", mechanism2d);
    controller.setTolerance(GOAL_POSITION_TOLERANCE);
  }

  /** Returns elevator height. */
  public double getHeight() {
    return this.currentState.position;
  }

  @Override
  public void disable() {
    mainMotor.disable();
    lastState = new TrapezoidProfile.State();
    collisionTimer.stop();
    collisionTimer.reset();
    isSeekingGoal = false;
    logIsSeekingGoal.append(false);
  }

  /**
   * Sets elevator goal level.
   *
   * @param level
   */
  public void setGoalPosition(ElevatorLevel level) {
    setGoalPosition(level.getElevatorHeight(), level.getArmOffset());
  }

  /**
   * Sets elevator goal height in meters.
   *
   * @param height
   */
  private void setGoalPosition(double height, double armOffset) {
    isSeekingGoal = true;
    goalState.position = height;
    goalState.velocity = 0;
    this.armOffset = armOffset;
    lastState = currentState;

    controller.setPID(KP.getValue(), KI.getValue(), KD.getValue());

    logIsSeekingGoal.append(true);
    logGoalPosition.append(height);
    logGoalVelocity.append(0);
  }

  /** Returns whether the elevator is at goal position. */
  public boolean atGoalPosition() {
    return controller.atGoal();
  }

  /** Returns whether the elevator is above the arm position. */
  public boolean aboveArmPosition() {
    return goalState.position - armOffset <= currentState.position;
  }

  private void updateSensorState() {
    if (RobotBase.isReal()) {
      currentState.position = encoder.getPosition();
      currentState.velocity = encoder.getVelocity();
    } else {
      currentState.position = simElevator.getPositionMeters();
      currentState.velocity = simElevator.getVelocityMetersPerSecond();
    }
    currentState.position += MIN_HEIGHT;

    checkError();
    atUpperLimit = currentState.position >= MAX_HEIGHT;
    atLowerLimit = currentState.position <= DISABLE_HEIGHT;

    elevatorMech2d.setLength(currentState.position);
    logCurrentPosition.append(currentState.position);
    logCurrentVelocity.append(currentState.velocity);
    logAtLowerLimit.append(atLowerLimit);
    logAtUpperLimit.append(atUpperLimit);
    logStatorCurrent.append(mainMotor.getStatorCurrent());
    logTorqueCurrent.append(mainMotor.getTorqueCurrent());
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    mainMotor.setIdleMode(idleMode);
    // We leave the follower in brake mode for more controlled decent when disabled.
    // follower.setIdleMode(idleMode);
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

  /** Returns if elevator is near the given level. */
  public boolean isNearestToLevel(ElevatorLevel level) {
    return MathUtil.isNear(level.getElevatorHeight(), getHeight(), 0.1);
  }

  @Override
  public void periodic() {
    updateSensorState();
    if (isSeekingGoal) {
      TrapezoidProfile.State desiredState =
          profile.calculate(RobotConstants.PERIODIC_INTERVAL, lastState, goalState);

      /*if (goalState.position == MIN_HEIGHT
                && Math.abs(currentState.velocity) < COLLISION_VELOCITY_THRESHOLD) {
              if (!collisionTimer.isRunning()) {
                collisionTimer.restart();
              } else if (collisionTimer.hasElapsed(COLLISION_DURATION)) {
                disable();
                return;
              }
            }
      */
      double feedforward = feedForward.calculate(desiredState.velocity);
      double pidOutput = controller.calculate(currentState.position, desiredState);

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
    } /*else {
        if (goalState.position == MIN_HEIGHT
            && !MathUtil.isNear(MIN_HEIGHT, currentState.position, STOWED_POSITION_TOLERANCE)
            && currentState.velocity == 0) {
          encoder.reset();
        }
      } */
  }

  @Override
  public void simulationPeriodic() {
    simElevator.setInputVoltage(currentVoltage);
    simElevator.update(0.020);
  }

  @Override
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab elevatorTab = Shuffleboard.getTab("Elevator");
    ShuffleboardLayout elevatorLayout =
        elevatorTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5);
    elevatorLayout.addDouble("Current Height", () -> this.currentState.position);
    elevatorLayout.addDouble("Current Velocity", () -> this.currentState.velocity);
    elevatorLayout.addDouble("Goal Height", () -> this.goalState.position);
    elevatorLayout.addDouble("Goal Velocity", () -> this.goalState.velocity);
    elevatorLayout
        .addBoolean("Is Enabled", () -> this.isSeekingGoal)
        .withWidget(BuiltInWidgets.kBooleanBox);
    elevatorLayout.add("Max Velocity", MAX_SPEED);
    elevatorLayout.add("Max Acceleration", MAX_ACCELERATION);

    ShuffleboardLayout controlLayout =
        elevatorTab.getLayout("Control", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 5);
    controlLayout.addDouble("Current Height", () -> this.currentState.position);
    GenericEntry elevatorHeight = controlLayout.add("Elevator Height", 0).getEntry();
    controlLayout.add(
        Commands.defer(
                () ->
                    Commands.runOnce(() -> this.setGoalPosition(elevatorHeight.getDouble(0), 0))
                        .until(() -> this.atGoalPosition()),
                Set.of(this))
            .withName("Set Height"));
    controlLayout.add(Commands.runOnce(() -> this.disable(), this).withName("Disable"));
    controlLayout.add(
        Commands.runOnce(() -> encoder.reset(), this)
            .ignoringDisable(true)
            .withName("Reset Encoder"));
    controlLayout.add(ElevatorCommands.stowElevator(this));
  }
}
