/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.BooleanLogEntry;
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
import frc.robot.Constants.RobotConstants;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.util.LimitSwitch;
import frc.robot.util.MotorController;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;

public class Elevator extends SubsystemBase {

  // physical parameters of the elevator
  private static final double GEAR_RATIO = 9.0;
  private static final double SPROCKET_DIAMETER = 0.3; // meters
  private static final double MASS = 2.0; // kilograms
  private static final double METERS_PER_REVOLUTION = (SPROCKET_DIAMETER * Math.PI) / GEAR_RATIO;

  // trapezoid profile values
  private static final DCMotor MOTOR_PARAMS = DCMotor.getKrakenX60(1);
  private static final double MAX_SPEED =
      (MOTOR_PARAMS.freeSpeedRadPerSec * SPROCKET_DIAMETER) / (GEAR_RATIO * 2 * Math.PI); // m/s
  private static final double MAX_ACCELERATION =
      (2 * MOTOR_PARAMS.stallTorqueNewtonMeters * GEAR_RATIO) / (SPROCKET_DIAMETER * MASS); // m/s^2
  private static final TrapezoidProfile.Constraints CONSTRAINTS =
      new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCELERATION);

  // feedforward constants
  private static final double KS = 0.15;
  private static final double KV = 12 / MAX_SPEED;
  private static final double KA = 12 / MAX_ACCELERATION;
  private static final double KG = 9.81 * KA;

  // feedback constants
  private static final double KP = 1.0;
  private static final double KI = 0;
  private static final double KD = 0;

  private MotorController mainMotor =
      new TalonFXAdapter(
          new TalonFX(RobotConstants.CAN.TalonFX.ELEVATOR_MAIN_MOTOR_ID, "rio"),
          false,
          true,
          METERS_PER_REVOLUTION);
  private MotorController follower =
      new TalonFXAdapter(
          mainMotor,
          new TalonFX(RobotConstants.CAN.TalonFX.ELEVATOR_FOLLOWER_MOTOR_ID, "rio"),
          false);
  private RelativeEncoder encoder = mainMotor.getEncoder();
  private LimitSwitch upperLimit = mainMotor.getForwardLimitSwitch();
  private LimitSwitch lowerLimit = mainMotor.getReverseLimitSwitch();

  private ElevatorSim simElevator =
      new ElevatorSim(MOTOR_PARAMS, GEAR_RATIO, MASS, SPROCKET_DIAMETER / 2, 0, 1, true, 0);

  private Mechanism2d mechanism2d = new Mechanism2d(0.5, 1.0);
  private MechanismRoot2d mechanismRoot2d = mechanism2d.getRoot("Elevator Root", 0, 0);
  private MechanismLigament2d elevatorMech2d =
      mechanismRoot2d.append(new MechanismLigament2d("Elevator", 0, 90));

  private final ElevatorFeedforward feedForward = new ElevatorFeedforward(KS, KG, KV, KA);
  private final TrapezoidProfile profile = new TrapezoidProfile(CONSTRAINTS);
  private final Timer timer = new Timer();

  private final ProfiledPIDController feedBack = new ProfiledPIDController(KP, KI, KD, CONSTRAINTS);

  private boolean isSeekingGoal;
  private final TrapezoidProfile.State currentState = new TrapezoidProfile.State();
  private final TrapezoidProfile.State goalState = new TrapezoidProfile.State();
  private boolean atUpperLimit;
  private boolean atLowerLimit;
  private double currentVoltage;

  private BooleanLogEntry logIsSeekingGoal =
      new BooleanLogEntry(DataLogManager.getLog(), "Elevator/isSeekingGoal");
  private DoubleLogEntry logCurrentVelocity =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/currentVelocity");
  private DoubleLogEntry logCurrentPosition =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/currentPosition");
  private DoubleLogEntry logGoalVelocity =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/goalVelocity");
  private DoubleLogEntry logGoalPosition =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/goalPosition");
  private DoubleLogEntry logCurrentVoltage =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/currentVoltage");
  private DoubleLogEntry logFeedForward =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/FeedForward");
  private DoubleLogEntry logPIDOutput =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/pidOutput");
  private BooleanLogEntry logAtUpperLimit =
      new BooleanLogEntry(DataLogManager.getLog(), "Elevator/atUpperLimit");
  private BooleanLogEntry logAtLowerLimit =
      new BooleanLogEntry(DataLogManager.getLog(), "Elevator/atLowerLimit");
  private DoubleLogEntry logDesiredPosition =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/desiredPosition");
  private DoubleLogEntry logDesiredVelocity =
      new DoubleLogEntry(DataLogManager.getLog(), "Elevator/desiredVelocity");

  /** Creates a new Elevator. */
  public Elevator() {
    updateSensorState();
    SmartDashboard.putData("Elevator Sim", mechanism2d);
  }

  public void disable() {
    mainMotor.disable();
    isSeekingGoal = false;
    timer.stop();
    logIsSeekingGoal.append(false);
  }

  public void setGoalPosition(ElevatorLevel level) {
    timer.reset();
    timer.start();
    isSeekingGoal = true;
    goalState.position = level.getHeight();
    goalState.velocity = 0;
    logIsSeekingGoal.append(true);
    logGoalPosition.append(level.getHeight());
    logGoalVelocity.append(0);
  }

  private void updateSensorState() {
    if (RobotBase.isReal()) {
      currentState.position = encoder.getPosition();
      currentState.velocity = encoder.getVelocity();
    } else {
      currentState.position = simElevator.getPositionMeters();
      currentState.velocity = simElevator.getVelocityMetersPerSecond();
    }
    atUpperLimit = upperLimit.isPressed();
    atLowerLimit = lowerLimit.isPressed();
    elevatorMech2d.setLength(currentState.position);
    logCurrentPosition.append(currentState.position);
    logCurrentVelocity.append(currentState.velocity);
    logAtLowerLimit.append(atLowerLimit);
    logAtUpperLimit.append(atUpperLimit);
  }

  @Override
  public void periodic() {
    updateSensorState();
    if (isSeekingGoal) {
      TrapezoidProfile.State desiredState = profile.calculate(timer.get(), currentState, goalState);
      logDesiredPosition.append(desiredState.position);
      logDesiredVelocity.append(desiredState.velocity);
      double feedforward = feedForward.calculate(desiredState.velocity);
      logFeedForward.append(feedforward);
      double pidOutput = feedBack.calculate(currentState.position, desiredState);
      logPIDOutput.append(pidOutput);
      currentVoltage = feedforward + pidOutput;
      if ((currentVoltage > 0 && atUpperLimit)) {
        currentVoltage = KG;
      }
      if ((currentVoltage < 0 && atLowerLimit)) {
        currentVoltage = 0;
      }
      mainMotor.setVoltage(currentVoltage);
      logCurrentVoltage.append(currentVoltage);
    }
  }

  @Override
  public void simulationPeriodic() {
    simElevator.setInputVoltage(currentVoltage);
    simElevator.update(0.020);
  }
}
