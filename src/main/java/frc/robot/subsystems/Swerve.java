/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveToPose;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.parameters.SwerveAngleEncoder;
import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.parameters.SwerveMotors;
import frc.robot.util.Gyro;
import frc.robot.util.MotorController;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.SwerveModuleVelocities;
import frc.robot.util.SwerveModuleVoltages;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

@RobotPreferencesLayout(
    groupName = "Drive",
    column = 1,
    row = 0,
    width = 3,
    height = 1,
    type = "Grid Layout",
    gridColumns = 3,
    gridRows = 1)
public class Swerve extends SubsystemBase implements ActiveSubsystem, ShuffleboardProducer {
  private static final DataLog LOG = DataLogManager.getLog();
  private static final Rotation2d ROTATE_180_DEGREES = Rotation2d.fromDegrees(180);

  @RobotPreferencesValue(column = 0, row = 0)
  public static RobotPreferences.EnumValue<SwerveDriveParameters> PARAMETERS =
      new RobotPreferences.EnumValue<SwerveDriveParameters>(
          "Drive", "Robot Base", SwerveDriveParameters.CompetitionBase2025);

  @RobotPreferencesValue(column = 1, row = 0)
  public static RobotPreferences.BooleanValue ENABLE_DRIVE_TAB =
      new RobotPreferences.BooleanValue("Drive", "Enable Tab", false);

  @RobotPreferencesValue(column = 2, row = 0)
  public static RobotPreferences.BooleanValue ENABLE_RUMBLE =
      new RobotPreferences.BooleanValue("Drive", "Enable Rumble", true);

  public static final double ROTATIONAL_KP = 1.0;
  public static final double DRIVE_KP = 1.0;

  // 4 pairs of motors for drive & steering.
  private final MotorController frontLeftDriveMotor =
      PARAMETERS.getValue().getMotorController(SwerveMotors.FrontLeftDrive);
  private final MotorController frontLeftSteeringMotor =
      PARAMETERS.getValue().getMotorController(SwerveMotors.FrontLeftSteering);

  private final MotorController frontRightDriveMotor =
      PARAMETERS.getValue().getMotorController(SwerveMotors.FrontRightDrive);
  private final MotorController frontRightSteeringMotor =
      PARAMETERS.getValue().getMotorController(SwerveMotors.FrontRightSteering);

  private final MotorController backLeftDriveMotor =
      PARAMETERS.getValue().getMotorController(SwerveMotors.BackLeftDrive);
  private final MotorController backLeftSteeringMotor =
      PARAMETERS.getValue().getMotorController(SwerveMotors.BackLeftSteering);

  private final MotorController backRightDriveMotor =
      PARAMETERS.getValue().getMotorController(SwerveMotors.BackRightDrive);
  private final MotorController backRightSteeringMotor =
      PARAMETERS.getValue().getMotorController(SwerveMotors.BackRightSteering);

  // 4 CANcoders for the steering angle.
  private final CANcoder frontLeftAngle =
      PARAMETERS.getValue().getAngleEncoder(SwerveAngleEncoder.FrontLeft);
  private final CANcoder frontRightAngle =
      PARAMETERS.getValue().getAngleEncoder(SwerveAngleEncoder.FrontRight);
  private final CANcoder backLeftAngle =
      PARAMETERS.getValue().getAngleEncoder(SwerveAngleEncoder.BackLeft);
  private final CANcoder backRightAngle =
      PARAMETERS.getValue().getAngleEncoder(SwerveAngleEncoder.BackRight);

  private final SwerveModule frontLeftModule =
      createSwerveModule(frontLeftDriveMotor, frontLeftSteeringMotor, frontLeftAngle, "Front Left");
  private final SwerveModule frontRightModule =
      createSwerveModule(
          frontRightDriveMotor, frontRightSteeringMotor, frontRightAngle, "Front Right");
  private final SwerveModule backLeftModule =
      createSwerveModule(backLeftDriveMotor, backLeftSteeringMotor, backLeftAngle, "Back Left");
  private final SwerveModule backRightModule =
      createSwerveModule(backRightDriveMotor, backRightSteeringMotor, backRightAngle, "Back Right");

  private final SwerveModule[] modules = {
    frontLeftModule, frontRightModule, backLeftModule, backRightModule
  };

  private final Gyro gyro = PARAMETERS.getValue().getGyro();
  private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  private final SwerveDriveKinematics kinematics = PARAMETERS.getValue().getKinematics();

  private final SwerveDrive drivetrain;
  private final SwerveDrivePoseEstimator odometry;

  // The current sensor state updated by the periodic method.
  private double rawOrientation; // The raw gyro orientation in radians.
  private double rawOrientationOffset; // The offset to the corrected orientation in radians.
  private Rotation2d orientation = Rotation2d.kZero;
  private Pose2d lastVisionMeasurement = new Pose2d();
  private Supplier<Optional<Rotation2d>> targetOrientationSupplier = () -> Optional.empty();
  private double acceleration = 0;

  private StructLogEntry<Pose2d> poseLog =
      StructLogEntry.create(LOG, "/Swerve/Pose", Pose2d.struct);
  private DoubleLogEntry rawOrientationLog = new DoubleLogEntry(LOG, "/Swerve/rawOrientation");
  private DoubleLogEntry rawOrientationOffsetLog =
      new DoubleLogEntry(LOG, "/Swerve/rawOrientationOffset");
  private DoubleLogEntry accelerationLog = new DoubleLogEntry(LOG, "/Swerve/acceleration");

  /**
   * Creates a {@link SwerveModule} object and intiailizes its motor controllers.
   *
   * @param driveMotor The drive motor controller.
   * @param steeringMotor The steering motor controller.
   * @param wheelAngle An absolute encoder that measures the wheel angle.
   * @param name The name of the module.
   * @return An initialized {@link SwerveModule} object.
   */
  private static SwerveModule createSwerveModule(
      MotorController driveMotor, MotorController steeringMotor, CANcoder wheelAngle, String name) {

    RelativeEncoder driveEncoder = driveMotor.getEncoder();
    StatusSignal<Angle> wheelOrientation = wheelAngle.getAbsolutePosition();
    StatusSignal<AngularVelocity> angularVelocity = wheelAngle.getVelocity();

    return new SwerveModule(
        PARAMETERS.getValue(),
        driveMotor,
        driveEncoder::getPosition,
        driveEncoder::getVelocity,
        steeringMotor,
        () -> new Rotation2d(wheelOrientation.refresh().getValue().in(Units.Radians)),
        () -> angularVelocity.refresh().getValue().in(Units.RadiansPerSecond),
        name);
  }

  /** Creates a new Swerve. */
  public Swerve() {
    initializeSensorState();

    drivetrain = new SwerveDrive(PARAMETERS.getValue(), modules, () -> getOrientation());
    odometry =
        new SwerveDrivePoseEstimator(
            kinematics, getOrientation(), drivetrain.getModulesPositions(), new Pose2d());
  }

  /** Initializes the sensor state. */
  private void initializeSensorState() {
    gyro.reset();
    updateSensorState();
  }

  /**
   * Updates the sensor state.
   *
   * <p>This method **MUST* be called by the {@link #periodic()} method to ensure the sensor state
   * is up to date.
   */
  private void updateSensorState() {
    double rawGyro = gyro.getAngle();
    rawOrientation = rawGyro;
    rawOrientationLog.append(Math.toDegrees(rawGyro));
    orientation = new Rotation2d(MathUtil.angleModulus(rawOrientation + rawOrientationOffset));

    double accelerationX = accelerometer.getX();
    double accelerationY = accelerometer.getY();
    acceleration = Math.hypot(accelerationX, accelerationY);
    accelerationLog.append(acceleration);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)} */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestamp) {
    odometry.addVisionMeasurement(visionMeasurement, timestamp);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)} */
  public void addVisionMeasurement(
      Pose2d visionMeasurment, double timestamp, Matrix<N3, N1> stdDevs) {
    odometry.addVisionMeasurement(visionMeasurment, timestamp, stdDevs);
    lastVisionMeasurement = visionMeasurment;
  }

  /*
   * Disable robot auto-orientation.
   */
  @Override
  public void disable() {
    disableAutoOrientation();
  }

  /**
   * Sets the absolute location of the target to keep the robot oriented to.
   *
   * @param orientationTarget Target we want to orient to.
   */
  public void enableAutoOrientationTarget(Translation2d orientationTarget) {
    this.targetOrientationSupplier =
        () ->
            Optional.of(
                orientationTarget
                    .minus(getPosition().getTranslation())
                    .getAngle()
                    .rotateBy(ROTATE_180_DEGREES));
  }

  /**
   * Enables auto orientation mode.
   *
   * @param targetOrientationSupplier Supplies the target orientation.
   */
  public void enableAutoOrientation(Supplier<Optional<Rotation2d>> targetOrientationSupplier) {
    this.targetOrientationSupplier = targetOrientationSupplier;
  }

  /** Clears the orientation target. */
  public void disableAutoOrientation() {
    targetOrientationSupplier = () -> Optional.empty();
  }

  /**
   * Returns the desired orientation when an orientation target is set.
   *
   * @return Returns an Optional<Rotation2d> when an orientation target is set. Otherwise, this
   *     method returns Optional.empty().
   */
  public Optional<Rotation2d> getTargetOrientation() {
    return targetOrientationSupplier.get();
  }

  /**
   * Returns the maximum drive speed in m/s of a swerve module.
   *
   * @return The maximum drive speed.
   */
  public static double getMaxSpeed() {
    return PARAMETERS.getValue().getMaxDriveSpeed();
  }

  /**
   * Returns the maximum drive acceleration in m/s^2 of a swerve module.
   *
   * @return The maximum drive acceleration.
   */
  public static double getMaxAcceleration() {
    return PARAMETERS.getValue().getMaxDriveAcceleration();
  }

  /** Gets acceleration in g with an range of +/- 8 g's */
  public double getAcceleration() {
    return acceleration;
  }

  /**
   * Returns the swerve drive kinematics for this subsystem.
   *
   * @return The swerve drive kinematics.
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns a {@link SwerveDriveKinematicsConstraint} object used to enforce swerve drive
   * kinematics constraints when following a trajectory.
   *
   * @return A {@link SwerveDriveKinematicsConstraint} object used to enforce swerve drive
   *     kinematics constraints when following a trajectory.
   */
  public static SwerveDriveKinematicsConstraint getKinematicsConstraint() {
    return PARAMETERS.getValue().getKinematicsConstraint();
  }

  /**
   * Returns the drive constraints.
   *
   * @return The drive constraints.
   */
  public TrapezoidProfile.Constraints getDriveConstraints() {
    return new TrapezoidProfile.Constraints(getMaxSpeed(), getMaxAcceleration());
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   * constraints on the {@link ProfiledPIDController} used to reach the goal robot orientation.
   *
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   *     constraints on the controller used to reach the goal robot orientation.
   */
  public static TrapezoidProfile.Constraints getRotationalConstraints() {
    return PARAMETERS.getValue().getRotationalConstraints();
  }

  /**
   * Return the wheel base radius in meters.
   *
   * @return The wheel base radius in meters.
   */
  public static double getWheelBaseRadius() {
    return PARAMETERS.getValue().getWheelBaseRadius();
  }

  /**
   * Creates a HolonomicDriveController for the subsystem.
   *
   * @return A HolonomicDriveController.
   */
  public HolonomicDriveController createDriveController() {
    ProfiledPIDController thetaController =
        new ProfiledPIDController(ROTATIONAL_KP, 0.0, 0.0, getRotationalConstraints());

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new HolonomicDriveController(
        new PIDController(DRIVE_KP, 0.0, 0.0),
        new PIDController(DRIVE_KP, 0.0, 0.0),
        thetaController);
  }

  /**
   * Drives the robot based on joystick inputs.
   *
   * @param xSpeed Speed of the robot in the x direction.
   * @param ySpeed Speed of the robot in the y direction.
   * @param rSpeed Rotation speed of the robot.
   * @param fieldRelative Whether the x and y values are relative to field.
   */
  public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative) {
    drivetrain.drive(xSpeed, ySpeed, rSpeed, fieldRelative);
  }

  /**
   * Sets the current module's states based on the chassis speed.
   *
   * @param speeds The chassis speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    drivetrain.setChassisSpeeds(speeds);
  }

  /**
   * Returns the current chassis speed.
   *
   * @return The chassis speed.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getChassisSpeeds();
  }

  public SwerveModuleState[] getModuleStates() {
    return drivetrain.getModuleStates();
  }

  public SwerveModulePosition[] getModulePositions() {
    return drivetrain.getModulesPositions();
  }

  /**
   * Returns the swerve module velocities.
   *
   * @return The swerve module velocities.
   */
  public SwerveModuleVelocities[] getModuleVelocities() {
    return drivetrain.getModuleVelocities();
  }

  /**
   * Sets the swerve module states.
   *
   * @param states An array of four {@link SwerveModuleState} objects in the order: front left,
   *     front right, back left, back right
   */
  public void setModuleStates(SwerveModuleState[] states) {
    drivetrain.setModuleStates(states);
  }

  /**
   * Gets the module motor voltages.
   *
   * @return The module motor
   */
  public SwerveModuleVoltages[] getModuleVoltages() {
    return drivetrain.getModuleVoltages();
  }

  /**
   * Sets the module motor voltages.
   *
   * @param moduleVoltages The module motor voltages.
   */
  public void setModuleVoltages(SwerveModuleVoltages[] moduleVoltages) {
    drivetrain.setModuleVoltages(moduleVoltages);
  }

  // Stops motors from the subsystem - may need to remove this (not sure - Om)
  public void stopMotors() {
    drivetrain.stopMotor();
  }

  /**
   * Resets the robots position on the field.
   *
   * @param desiredPosition Sets the initial position.
   */
  public void resetPosition(Pose2d desiredPosition) {
    orientation = desiredPosition.getRotation();
    rawOrientationOffset = MathUtil.angleModulus(orientation.getRadians() - rawOrientation);
    rawOrientationOffsetLog.append(Math.toDegrees(rawOrientationOffset));

    odometry.resetPosition(getOrientation(), drivetrain.getModulesPositions(), desiredPosition);
  }

  /** Resets the orientation the robot. */
  public void resetOrientation(Rotation2d orientation) {
    Pose2d currentPos = odometry.getEstimatedPosition();
    Pose2d newPos2d = new Pose2d(currentPos.getTranslation(), orientation);
    resetPosition(newPos2d);
  }

  /**
   * Return current position & orientation of the robot on the field.
   *
   * @return The current position and orientation of the robot.
   */
  public Pose2d getPosition() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Returns the current position and orienation of the robot on the field in 3-dimensional space.
   *
   * @return The current position and orientation in 3-dimensional space.
   */
  public Pose3d getPosition3d() {
    Pose2d robotPose2d = getPosition();

    return new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0.0,
        new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
  }

  /**
   * Returns the field orientation of the robot as a {@link Rotation2d} object.
   *
   * @return Gets the field orientation of the robot.
   */
  public Rotation2d getOrientation() {
    return orientation;
  }

  @Override
  public void periodic() {
    // Read sensors to update subsystem state.
    updateSensorState();

    // Update the current module state.
    drivetrain.periodic();

    // Update odometry last since this relies on the subsystem sensor and module
    // states.
    odometry.update(getOrientation(), drivetrain.getModulesPositions());

    // Send the robot and module location to the logger
    Pose2d robotPose = getPosition();

    poseLog.append(robotPose);
  }

  public void setIdleMode(MotorIdleMode idleMode) {
    frontLeftDriveMotor.setIdleMode(idleMode);
    frontRightDriveMotor.setIdleMode(idleMode);
    backLeftDriveMotor.setIdleMode(idleMode);
    backRightDriveMotor.setIdleMode(idleMode);

    frontLeftSteeringMotor.setIdleMode(idleMode);
    frontRightSteeringMotor.setIdleMode(idleMode);
    backLeftSteeringMotor.setIdleMode(idleMode);
    backRightSteeringMotor.setIdleMode(idleMode);
  }

  /** Adds a tab for swerve drive in Shuffleboard. */
  @Override
  public void addShuffleboardTab() {
    if (ENABLE_DRIVE_TAB.getValue()) {
      ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Drive");

      drivetrain.addShuffleboardLayouts(swerveDriveTab);

      ShuffleboardLayout odometryLayout =
          swerveDriveTab
              .getLayout("Odometry", BuiltInLayouts.kList)
              .withPosition(6, 0)
              .withSize(4, 4);

      odometryLayout
          .add(
              "Orientation",
              new Sendable() {
                @Override
                public void initSendable(SendableBuilder builder) {
                  builder.setSmartDashboardType("Gyro");
                  builder.addDoubleProperty("Value", () -> -getOrientation().getDegrees(), null);
                }
              })
          .withWidget(BuiltInWidgets.kGyro)
          .withPosition(0, 0);

      ShuffleboardLayout positionLayout =
          odometryLayout
              .getLayout("Position", BuiltInLayouts.kGrid)
              .withProperties(Map.of("Number of columns", 3, "Number of rows", 2));

      positionLayout
          .addDouble("X", () -> odometry.getEstimatedPosition().getX())
          .withPosition(0, 0)
          .withWidget(BuiltInWidgets.kTextView);
      positionLayout
          .addDouble("Y", () -> odometry.getEstimatedPosition().getY())
          .withPosition(1, 0)
          .withWidget(BuiltInWidgets.kTextView);

      positionLayout
          .addDouble("est. X", () -> lastVisionMeasurement.getX())
          .withPosition(0, 1)
          .withWidget(BuiltInWidgets.kTextView);
      positionLayout
          .addDouble("est. Y", () -> lastVisionMeasurement.getY())
          .withPosition(1, 1)
          .withWidget(BuiltInWidgets.kTextView);
      positionLayout
          .addDouble("est. angle", () -> lastVisionMeasurement.getRotation().getDegrees())
          .withPosition(2, 1)
          .withWidget(BuiltInWidgets.kTextView);
      ShuffleboardLayout driveStraight =
          swerveDriveTab
              .getLayout("Drive Straight", BuiltInLayouts.kList)
              .withPosition(0, 4)
              .withSize(4, 2);
      GenericEntry distance = driveStraight.add("Distance", 0).getEntry();
      driveStraight.add(
          Commands.defer(
                  () -> new DriveToPose(this, distance.getDouble(0), Swerve.getMaxSpeed() * 0.25),
                  Set.of(this))
              .withName("Drive Straight"));
    }
  }
}
