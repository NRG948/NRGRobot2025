/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;
import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.util.SwerveModuleVelocities;
import frc.robot.util.SwerveModuleVoltages;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Manages the drive and steering motors of a single swerve drive module.
 *
 * <p>This class uses a combination feedback (i.e. PID) and feedforward control to achieve desired
 * translational (i.e. drive) and rotational (i.e. steering) velocities. For more information, see
 * the <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html">
 * Introduction to PID</a> and <a href=
 * "https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html">
 * Introduction to DC Motor Feedforward</a> articles of the WPILib documentation.
 */
public class SwerveModule {
  private static final DataLog LOG = DataLogManager.getLog();

  private final MotorController driveMotor;
  private final DoubleSupplier positionSupplier;
  private final MotorController steeringMotor;
  private final DoubleSupplier velocitySupplier;
  private final Supplier<Rotation2d> wheelAngleSupplier;
  private final DoubleSupplier wheelAngleVelocitySupplier;
  private final String name;
  private final double wheelDiameter;

  // models motors mathematically, calculates voltage needed
  private final SimpleMotorFeedforward driveFeedForward;
  private final SimpleMotorFeedforward steeringFeedForward;

  private final PIDController drivePID;
  private final ProfiledPIDController steeringPID;

  // The last motor voltages applied.
  private double driveVoltage;
  private double steeringVoltage;

  // The current supplied state updated by the periodic method.
  private final SwerveModuleState state = new SwerveModuleState();
  private final SwerveModulePosition position = new SwerveModulePosition();
  private final SwerveModuleVelocities velocities = new SwerveModuleVelocities();

  private final DoubleLogEntry driveSpeedLog;
  private final DoubleLogEntry positionLog;
  private final DoubleLogEntry wheelAngleLog;
  private final DoubleLogEntry wheelAngleVelocityLog;
  private final DoubleLogEntry stateVelocityLog;
  private final DoubleLogEntry stateWheelAngleLog;
  private final DoubleLogEntry driveVoltageLog;
  private final DoubleLogEntry steeringVoltageLog;

  // Simulation support.
  private double simVelocity;
  private double simPosition;
  private Rotation2d simWheelAngle = Rotation2d.kZero;
  private double simWheelAngleVelocity;
  private FlywheelSim simDriveMotor;
  private FlywheelSim simSteeringMotor;

  /**
   * Constructs the swerve module.
   *
   * @param parameters A {@link SwerveDriveParameters} object providing information on the physical
   *     swerve drive characteristics.
   * @param driveMotor The drive motor controller.
   * @param position Supplies the position in meters.
   * @param velocity Supplies velocity in meters per second.
   * @param steeringMotor The steering motor controller.
   * @param wheelAngle Supplies the wheel angle.
   * @param wheelAngleVelocity Supplies the wheel angle velocity in radians per second.
   * @param name The name of the module.
   */
  public SwerveModule(
      SwerveDriveParameters parameters,
      MotorController driveMotor,
      DoubleSupplier position,
      DoubleSupplier velocity,
      MotorController steeringMotor,
      Supplier<Rotation2d> wheelAngle,
      DoubleSupplier wheelAngleVelocity,
      String name) {
    boolean realRobot = Robot.isReal();

    this.driveMotor = driveMotor;
    this.steeringMotor = steeringMotor;
    this.wheelAngleSupplier = realRobot ? wheelAngle : () -> this.simWheelAngle;
    this.wheelAngleVelocitySupplier =
        realRobot ? wheelAngleVelocity : () -> this.simWheelAngleVelocity;
    this.positionSupplier = realRobot ? position : () -> this.simPosition;
    this.velocitySupplier = realRobot ? velocity : () -> this.simVelocity;
    this.name = name;
    this.wheelDiameter = parameters.getModuleParameters().getWheelDiameter();

    this.driveSpeedLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/driveSpeed", name));
    this.positionLog = new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/position", name));
    this.wheelAngleLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/wheelAngle", name));
    this.wheelAngleVelocityLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/wheelAngleVelocity", name));
    this.stateVelocityLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/stateVelocity", name));
    this.stateWheelAngleLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/stateWheelAngle", name));
    this.driveVoltageLog =
        new DoubleLogEntry(LOG, String.format("/SwerveModule/%s/driveVoltage", name));
    this.steeringVoltageLog =
        new DoubleLogEntry(LOG, String.format("SwerveModule/%s/steeringVoltage", name));

    initializeSuppliedState();

    this.driveFeedForward =
        new SimpleMotorFeedforward(
            parameters.getDriveKs(), parameters.getDriveKv(), parameters.getDriveKa());
    this.steeringFeedForward =
        new SimpleMotorFeedforward(
            parameters.getSteeringKs(), parameters.getSteeringKv(), parameters.getSteeringKa());

    this.drivePID = new PIDController(0, 0, 0.0);

    this.steeringPID = new ProfiledPIDController(3.5, 0, 0.0, parameters.getSteeringConstraints());
    this.steeringPID.enableContinuousInput(-Math.PI, Math.PI);
    this.steeringPID.setTolerance(Math.toRadians(1.0));
    this.steeringPID.reset(getPosition().angle.getRadians());

    this.simDriveMotor =
        new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(parameters.getDriveKv(), parameters.getDriveKa()),
            DCMotor.getFalcon500(1),
            parameters.getModuleParameters().getDriveGearRatio());
    this.simSteeringMotor =
        new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(
                parameters.getSteeringKv(), parameters.getSteeringKa()),
            DCMotor.getFalcon500(1),
            parameters.getModuleParameters().getSteeringGearRatio());
  }

  /** Initializes the supplied state. */
  private void initializeSuppliedState() {
    updateSuppliedState();
  }

  /**
   * Updates the supplied state.
   *
   * <p>This method **MUST* be called by the {@link #periodic()} method to ensure the supplied state
   * is up to date for subsequent use.
   */
  private void updateSuppliedState() {
    Rotation2d wheelAngle = wheelAngleSupplier.get();
    double velocity = velocitySupplier.getAsDouble();
    double wheelAngleVelocity = wheelAngleVelocitySupplier.getAsDouble();
    double position = positionSupplier.getAsDouble();

    this.position.distanceMeters = position;
    this.position.angle = wheelAngle;

    this.state.speedMetersPerSecond = velocity;
    this.state.angle = wheelAngle;

    this.velocities.driveVelocity = velocity;
    this.velocities.steeringVelocity = wheelAngleVelocity;

    driveSpeedLog.append(velocity);
    positionLog.append(position);
    wheelAngleLog.append(wheelAngle.getDegrees());
    wheelAngleVelocityLog.append(wheelAngleVelocity);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param newState The desired state w/ speed and angle
   */
  public void setState(SwerveModuleState newState) {
    // Optimize the state to avoid spinning further than 90 degrees
    Rotation2d currentAngle = getWheelRotation2d();
    newState.optimize(currentAngle);
    newState.cosineScale(currentAngle);

    stateVelocityLog.append(newState.speedMetersPerSecond);
    stateWheelAngleLog.append(newState.angle.getDegrees());

    // Calculate the drive motor voltage using PID and FeedForward
    double driveOutput =
        drivePID.calculate(state.speedMetersPerSecond, newState.speedMetersPerSecond);
    double driveFeedForward = this.driveFeedForward.calculate(newState.speedMetersPerSecond);

    // Calculate the steering motor voltage using PID and FeedForward
    double steeringOutput =
        steeringPID.calculate(currentAngle.getRadians(), newState.angle.getRadians());
    double steeringFeedForward =
        this.steeringFeedForward.calculate(steeringPID.getSetpoint().velocity);

    // Sets voltages of motors
    driveVoltage = driveOutput + driveFeedForward;
    steeringVoltage = steeringOutput + steeringFeedForward;

    setMotorVoltages(driveVoltage, steeringVoltage);
  }

  /**
   * Get the drive and steering motor voltages.
   *
   * @return The motor voltages.
   */
  public SwerveModuleVoltages getMotorVoltages() {
    double driveVoltage = this.driveMotor.get() * RobotController.getBatteryVoltage();
    double steeringVoltage = this.steeringMotor.get() * RobotController.getBatteryVoltage();

    return new SwerveModuleVoltages(driveVoltage, steeringVoltage);
  }

  /**
   * Sets the drive and steering motor voltages.
   *
   * @param driveVoltage The drive motor voltage.
   * @param steeringVoltage The steering motor voltage.
   */
  public void setMotorVoltages(double driveVoltage, double steeringVoltage) {
    this.driveMotor.setVoltage(driveVoltage);
    this.steeringMotor.setVoltage(steeringVoltage);
    this.driveVoltageLog.append(driveVoltage);
    this.steeringVoltageLog.append(steeringVoltage);
  }

  /**
   * Sets the drive and steering motor voltages.
   *
   * @param moduleVoltages The motor voltages.
   */
  public void setMotorVoltages(SwerveModuleVoltages moduleVoltages) {
    setMotorVoltages(moduleVoltages.driveVoltage, moduleVoltages.steeringVoltage);
  }

  /** Stops the drive and steering motors. */
  public void stopMotors() {
    driveMotor.stopMotor();
    steeringMotor.stopMotor();
  }

  /**
   * Returns the current module state describing the wheel velocity and angle.
   *
   * @return The current module state.
   */
  public SwerveModuleState getState() {
    return state;
  }

  /**
   * The position of the wheel on its axis of travel.
   *
   * @return The position of the swerve module.
   */
  public SwerveModulePosition getPosition() {
    return position;
  }

  /**
   * Returns the current module velocities.
   *
   * @return The current module velocities.
   */
  public SwerveModuleVelocities getVelocities() {
    return velocities;
  }

  /**
   * Returns the current wheel orientation.
   *
   * @return The current wheel orientation.
   */
  public Rotation2d getWheelRotation2d() {
    return position.angle;
  }

  /**
   * This method is called periodically by the {@link Swerve}. It is used to update module-specific
   * state.
   */
  public void periodic() {
    updateSuppliedState();
  }

  /**
   * This method is called periodically by the {@link Swerve}. It is used to update module-specific
   * simulation state.
   */
  public void simulationPeriodic() {
    simDriveMotor.setInputVoltage(driveVoltage);
    simSteeringMotor.setInputVoltage(steeringVoltage);

    simDriveMotor.update(Robot.kDefaultPeriod);
    simSteeringMotor.update(Robot.kDefaultPeriod);

    simVelocity = (simDriveMotor.getAngularVelocityRadPerSec() * wheelDiameter) / 2;
    simPosition += simVelocity * Robot.kDefaultPeriod;

    simWheelAngleVelocity = simSteeringMotor.getAngularVelocityRadPerSec();
    simWheelAngle =
        new Rotation2d(simWheelAngle.getRadians() + (simWheelAngleVelocity * Robot.kDefaultPeriod));
  }

  /**
   * Adds the SwerveModule layout to the Shuffleboard tab.
   *
   * @param tab The Shuffleboard tab to add the layout.
   * @return The SwerveModule layout.
   */
  public ShuffleboardLayout addShuffleboardLayout(ShuffleboardTab tab) {
    ShuffleboardLayout moduleLayout =
        tab.getLayout(name, BuiltInLayouts.kGrid)
            .withProperties(
                Map.of("Number of columns", 2, "Number of rows", 1, "Label position", "HIDDEN"));

    moduleLayout
        .add(
            "Rotation",
            new Sendable() {

              @Override
              public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Gyro");
                builder.addDoubleProperty("Value", () -> -getPosition().angle.getDegrees(), null);
              }
            })
        .withWidget(BuiltInWidgets.kGyro)
        .withPosition(0, 0);

    ShuffleboardLayout translationLayout =
        moduleLayout
            .getLayout("State", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 1, "Number of rows", 3))
            .withPosition(1, 0);
    translationLayout
        .addDouble("Position", () -> getPosition().distanceMeters)
        .withPosition(0, 0)
        .withWidget(BuiltInWidgets.kTextView);
    translationLayout
        .addDouble("Velocity", () -> getState().speedMetersPerSecond)
        .withPosition(0, 1)
        .withWidget(BuiltInWidgets.kTextView);
    translationLayout
        .addDouble("Angular Velocity", () -> getVelocities().steeringVelocity)
        .withPosition(0, 2)
        .withWidget(BuiltInWidgets.kTextView);

    return moduleLayout;
  }
}
