/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import static frc.robot.Constants.RobotConstants.COMPETITION_ROBOT_MASS_KG;
import static frc.robot.Constants.RobotConstants.PRACTICE_ROBOT_MASS_KG;
import static frc.robot.parameters.MotorParameters.Falcon500;
import static frc.robot.parameters.MotorParameters.KrakenX60;
import static frc.robot.parameters.MotorParameters.NeoV1_1;
import static frc.robot.parameters.SwerveModuleParameters.MK4I_L2_PLUS;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Gyro;
import frc.robot.util.MotorController;
import frc.robot.util.MotorDirection;
import frc.robot.util.NavXGyro;
import frc.robot.util.Pigeon2Gyro;

/** An enum representing the properties for the swerve drive base of a specific robot instance. */
public enum SwerveDriveParameters {
  CompetitionBase2024(
      58.5,
      Units.inchesToMeters(22.755), // .578m  (22.755)
      Units.inchesToMeters(20.59), // .523m (20.59)
      MK4I_L2_PLUS,
      Falcon500,
      NeoV1_1,
      // for 2025 testing, the camera is treated as the front of the robot
      new int[] {13, 14, 8, 9, 19, 18, 6, 7}, // drive, steer motor controller CAN IDs
      new int[] {34, 33, 32, 31}, // CANCoder CAN IDs
      new double[] {22.85, 300.85, 324.67, 349.10},
      0.15,
      0.15,
      false,
      0),
  PracticeBase2025(
      PRACTICE_ROBOT_MASS_KG,
      0.602,
      0.602,
      MK4I_L2_PLUS,
      KrakenX60,
      NeoV1_1,
      new int[] {17, 19, 18, 20, 1, 2, 9, 10}, // drive, steer motor controller CAN IDs
      new int[] {33, 34, 31, 32}, // CANCoder CAN IDs
      new double[] {136.14, 169.98, 166.82, 184.31}, // CANCoder offsets
      0.0656,
      0.2057,
      true,
      21),
  CompetitionBase2025(
      COMPETITION_ROBOT_MASS_KG,
      0.602,
      0.602,
      MK4I_L2_PLUS,
      KrakenX60,
      NeoV1_1,
      new int[] {15, 14, 13, 6, 2, 1, 8, 7}, // drive, steer motor controller CAN IDs
      new int[] {31, 32, 33, 34}, // CANCoder CAN IDs
      new double[] {159.79, 109.25, 170.95, 4.66}, // CANCoder offsets
      0.0656,
      0.2057,
      true,
      21);

  public static class Constants {
    /**
     * A scaling factor used to adjust from theoretical maximums given that any physical system
     * generally cannot achieve them.
     */
    public static final double SCALE_FACTOR = 0.8;
  }

  private final double robotMass;
  private final double wheelDistanceX;
  private final double wheelDistanceY;
  private final SwerveModuleParameters moduleParams;
  private final MotorParameters driveMotor;
  private final MotorParameters steeringMotor;
  private final int[] motorIds;
  private final int[] angleEncoderIds;
  private final double[] angleOffset;
  private final boolean usesPigeon;
  private final int pigeonID;

  private final double maxDriveSpeed;
  private final double maxDriveAcceleration;

  private final FeedforwardConstants driveFeedforward;

  private final double maxSteeringSpeed;
  private final double maxSteeringAcceleration;

  private final FeedforwardConstants steeringFeedforward;

  private final double maxRotationalSpeed;
  private final double maxRotationalAcceleration;

  private final Translation2d[] wheelPositions;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveKinematicsConstraint kinematicsConstraint;
  private final TrapezoidProfile.Constraints steeringConstraints;
  private final TrapezoidProfile.Constraints robotRotationalConstraints;

  /**
   * Constructs an instance of this enum.
   *
   * <p><b>NOTE:</b> The distance between wheels are expressed in the NWU coordinate system relative
   * to the robot frame as shown below.
   *
   * <p>
   *
   * <pre>
   * <code>
   *            ^
   * +--------+ | x
   * |O      O| |
   * |        | |
   * |        | |
   * |O      O| |
   * +--------+ |
   *            |
   * <----------+
   *  y       (0,0)
   * </code>
   * </pre>
   *
   * @param robotMass The mass of the robot in Kg including bumpers and battery.
   * @param wheelDistanceX The distance between the wheels along the X axis in meters.
   * @param wheelDistanceY The distance between the wheels along the Y axis in meters.
   * @param moduleParams The swerve module used by the robot.
   * @param driveMotor The motor used by swerve module drive on the robot.
   * @param steeringMotor The motor used by the swerve module steering on the robot.
   * @param motorIds An array containing the CAN IDs of the swerve module drive motors in the order
   *     front left drive and steering, front right drive and steering, back left drive and
   *     steering, back right drive and steering.
   * @param angleEncoderIds An array containing the CAN IDs of the swerve module angle encoders in
   *     the order front left, front right, back left, back right.
   * @param angleOffset An array containing the zero point offsets for the swerve module angle
   *     encoders in the order front left, front right, back left, back right.
   * @param driveFeedforward The drive feedforward constants.
   * @param steeringFeedforward The steering feedforward constants.
   * @param usesPigeon Whether the robot uses a Pigeon 2 or NavX MXP for its gyro.
   * @param pigeonID The CAN ID for the Pigeon 2 gyro if present.
   */
  private SwerveDriveParameters(
      double robotMass,
      double wheelDistanceX,
      double wheelDistanceY,
      SwerveModuleParameters moduleParams,
      MotorParameters driveMotor,
      MotorParameters steeringMotor,
      int[] motorIds,
      int[] angleEncoderIds,
      double[] angleOffset,
      FeedforwardConstants driveFeedForward,
      FeedforwardConstants steeringFeedForward,
      boolean usesPigeon,
      int pigeonID) {
    this.robotMass = robotMass;
    this.wheelDistanceX = wheelDistanceX;
    this.wheelDistanceY = wheelDistanceY;
    this.moduleParams = moduleParams;
    this.driveMotor = driveMotor;
    this.steeringMotor = steeringMotor;
    this.motorIds = motorIds;
    this.angleEncoderIds = angleEncoderIds;
    this.angleOffset = angleOffset;
    this.driveFeedforward = driveFeedForward;
    this.steeringFeedforward = steeringFeedForward;
    this.usesPigeon = usesPigeon;
    this.pigeonID = pigeonID;

    double scaleFactor = Constants.SCALE_FACTOR;

    this.maxDriveSpeed = scaleFactor * this.moduleParams.calculateMaxDriveSpeed(this.driveMotor);
    this.maxDriveAcceleration =
        scaleFactor
            * this.moduleParams.calculateMaxDriveAcceleration(this.driveMotor, this.robotMass);

    this.maxSteeringSpeed =
        scaleFactor * this.moduleParams.calculateMaxSteeringSpeed(this.steeringMotor);
    this.maxSteeringAcceleration =
        scaleFactor
            * this.moduleParams.calculateMaxSteeringAcceleration(
                this.steeringMotor, this.robotMass);

    final double wheelTrackRadius = Math.hypot(this.wheelDistanceX, this.wheelDistanceY);

    this.maxRotationalSpeed = this.maxDriveSpeed / wheelTrackRadius;
    this.maxRotationalAcceleration = this.maxDriveAcceleration / wheelTrackRadius;

    this.wheelPositions =
        new Translation2d[] {
          new Translation2d(this.wheelDistanceX / 2.0, this.wheelDistanceY / 2.0),
          new Translation2d(this.wheelDistanceX / 2.0, -this.wheelDistanceY / 2.0),
          new Translation2d(-this.wheelDistanceX / 2.0, this.wheelDistanceY / 2.0),
          new Translation2d(-this.wheelDistanceX / 2.0, -this.wheelDistanceY / 2.0),
        };

    this.kinematics = new SwerveDriveKinematics(wheelPositions);
    this.kinematicsConstraint = new SwerveDriveKinematicsConstraint(kinematics, this.maxDriveSpeed);
    this.steeringConstraints =
        new TrapezoidProfile.Constraints(this.maxSteeringSpeed, this.maxSteeringAcceleration);
    this.robotRotationalConstraints =
        new TrapezoidProfile.Constraints(this.maxRotationalSpeed, this.maxRotationalAcceleration);
  }

  /**
   * Constructs an instance of this enum.
   *
   * <p><b>NOTE:</b> The distance between wheels are expressed in the NWU coordinate system relative
   * to the robot frame as shown below.
   *
   * <p>
   *
   * <pre>
   * <code>
   *            ^
   * +--------+ | x
   * |O      O| |
   * |        | |
   * |        | |
   * |O      O| |
   * +--------+ |
   *            |
   * <----------+
   *  y       (0,0)
   * </code>
   * </pre>
   *
   * @param robotMass The mass of the robot in Kg including bumpers and battery.
   * @param wheelDistanceX The distance between the wheels along the X axis in meters.
   * @param wheelDistanceY The distance between the wheels along the Y axis in meters.
   * @param swerveModule The swerve module used by the robot.
   * @param driveMotor The motor used by swerve module drive on the robot.
   * @param steeringMotor The motor used by the swerve module steering on the robot.
   * @param motorIds An array containing the CAN IDs of the swerve module drive motors in the order
   *     front left drive and steering, front right drive and steering, back left drive and
   *     steering, back right drive and steering.
   * @param angleEncoderIds An array containing the CAN IDs of the swerve module angle encoders in
   *     the order front left, front right, back left, back right.
   * @param angleOffset An array containing the zero point offsets for the swerve module angle
   *     encoders in the order front left, front right, back left, back right.
   * @param driveFeedforward The drive feedforward constants.
   * @param steeringKs The steering Ks feedforward constant.
   * @param usesPigeon Whether the robot uses a Pigeon 2 or NavX MXP for its gyro.
   * @param pigeonID The CAN ID for the Pigeon 2 gyro if present.
   */
  private SwerveDriveParameters(
      double robotMass,
      double wheelDistanceX,
      double wheelDistanceY,
      SwerveModuleParameters swerveModule,
      MotorParameters driveMotor,
      MotorParameters steeringMotor,
      int[] motorIds,
      int[] angleEncoderIds,
      double[] angleOffset,
      FeedforwardConstants driveFeedForward,
      double steeringkS,
      boolean usesPigeon,
      int pigeonID) {
    this(
        robotMass,
        wheelDistanceX,
        wheelDistanceY,
        swerveModule,
        driveMotor,
        steeringMotor,
        motorIds,
        angleEncoderIds,
        angleOffset,
        driveFeedForward,
        new CalculatedFeedforwardConstants(
            steeringkS,
            () -> swerveModule.calculateMaxSteeringSpeed(steeringMotor),
            () -> swerveModule.calculateMaxSteeringAcceleration(steeringMotor, robotMass)),
        usesPigeon,
        pigeonID);
  }

  /**
   * Constructs an instance of this enum.
   *
   * <p><b>NOTE:</b> The distance between wheels are expressed in the NWU coordinate system relative
   * to the robot frame as shown below.
   *
   * <p>
   *
   * <pre>
   * <code>
   *            ^
   * +--------+ | x
   * |O      O| |
   * |        | |
   * |        | |
   * |O      O| |
   * +--------+ |
   *            |
   * <----------+
   *  y       (0,0)
   * </code>
   * </pre>
   *
   * @param robotMass The mass of the robot in Kg.
   * @param wheelDistanceX The distance between the wheels along the X axis in meters.
   * @param wheelDistanceY The distance between the wheels along the Y axis in meters
   * @param swerveModule The swerve module used by the robot.
   * @param driveMotor The motor used by swerve module drive on the robot.
   * @param steeringMotor The motor used by the swerve module steering on the robot.
   * @param motorIds An array containing the CAN IDs of the swerve module drive motors in the order
   *     front left drive and steering, front right drive and steering, back left drive and
   *     steering, back right drive and steering.
   * @param angleEncoderIds An array containing the CAN IDs of the swerve module angle encoders in
   *     the order front left, front right, back left, back right.
   * @param angleOffset An array containing the zero point offsets for the swerve module angle
   *     encoders in the order front left, front right, back left, back right.
   * @param driveKs The drive kS constant.
   * @param steeringKs The steering kS constant.
   * @param usesPigeon Whether the robot uses a Pigeon 2 or NavX MXP for its gyro.
   * @param pigeonID The CAN ID for the Pigeon 2 gyro if present.
   */
  private SwerveDriveParameters(
      double robotMass,
      double wheelDistanceX,
      double wheelDistanceY,
      SwerveModuleParameters swerveModule,
      MotorParameters driveMotor,
      MotorParameters steeringMotor,
      int[] motorIds,
      int[] angleEncoderIds,
      double[] angleOffset,
      double drivekS,
      double steeringkS,
      boolean usesPigeon,
      int pigeonID) {
    this(
        robotMass,
        wheelDistanceX,
        wheelDistanceY,
        swerveModule,
        driveMotor,
        steeringMotor,
        motorIds,
        angleEncoderIds,
        angleOffset,
        new CalculatedFeedforwardConstants(
            drivekS,
            () -> swerveModule.calculateMaxDriveSpeed(driveMotor),
            () -> swerveModule.calculateMaxDriveAcceleration(driveMotor, robotMass)),
        new CalculatedFeedforwardConstants(
            steeringkS,
            () -> swerveModule.calculateMaxSteeringSpeed(steeringMotor),
            () -> swerveModule.calculateMaxSteeringAcceleration(steeringMotor, robotMass)),
        usesPigeon,
        pigeonID);
  }

  /**
   * Returns the mass of the robot in Kg.
   *
   * @return The mass of the robot in Kg.
   */
  public double getRobotMass() {
    return this.robotMass;
  }

  /**
   * Returns the maximum rotational speed of the robot in rad/s.
   *
   * @return The maximum rotation speed of the robot in rad/s.
   */
  public double getMaxRotationalSpeed() {
    return this.maxRotationalSpeed;
  }

  /**
   * Returns the maximum rotational acceleration of the robot in rad/s^2.
   *
   * @return The maximum rotation acceleration of the robot in rad/s^s.
   */
  public double getMaxRotationalAcceleration() {
    return this.maxRotationalAcceleration;
  }

  /**
   * Return the wheel base radius in meters.
   *
   * @return The wheel base radius in meters.
   */
  public double getWheelBaseRadius() {
    double halfWheelDistanceX = wheelDistanceX / 2;
    double halfWheelDistanceY = wheelDistanceY / 2;

    return Math.sqrt(
        halfWheelDistanceX * halfWheelDistanceX + halfWheelDistanceY * halfWheelDistanceY);
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   * constraints on the {@link ProfilePIDController} used to reach the goal robot orientation.
   *
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   *     constraints on the controller used to reach the goal robot orientation.
   */
  public TrapezoidProfile.Constraints getRotationalConstraints() {
    return this.robotRotationalConstraints;
  }

  /**
   * Returns the distance between the wheels along the X-axis in meters.
   *
   * @return The distance between the wheels along the X-axis in meters.
   */
  public double getWheelDistanceX() {
    return this.wheelDistanceX;
  }

  /**
   * Returns the distance between the wheels along the Y-axis in meters.
   *
   * @return The distance between the wheels along the Y-axis in meters.
   */
  public double getWheelDistanceY() {
    return this.wheelDistanceY;
  }

  /**
   * Returns the wheel positions relative to the robot center.
   *
   * @return The wheel positions relative to the robot center.
   */
  public Translation2d[] getWheelPositions() {
    return this.wheelPositions;
  }

  /**
   * Returns the wheel diameter in meters.
   *
   * @return The wheel diameter in meters.
   */
  public double getWheelDiameter() {
    return this.moduleParams.getWheelDiameter();
  }

  /**
   * Returns the drive gear ratio.
   *
   * @return The drive gear ratio.
   */
  public double getDriveGearRatio() {
    return this.moduleParams.getDriveGearRatio();
  }

  /**
   * Returns the steering gear ratio.
   *
   * @return The steering gear ratio.
   */
  public double getSteeringGearRatio() {
    return this.moduleParams.getSteeringGearRatio();
  }

  /**
   * Returns the swerve module used by the robot.
   *
   * @return The swerve module used by the robot.
   */
  public SwerveModuleParameters getModuleParameters() {
    return this.moduleParams;
  }

  /**
   * Returns the motor used by swerve module on the robot.
   *
   * @return The motor used by swerve module on the robot.
   */
  public MotorParameters getMotorParameters() {
    return this.driveMotor;
  }

  /**
   * Returns the CAN ids of the specified motor.
   *
   * @param motor The motor.
   * @return The CAN id.
   */
  public int getMotorId(SwerveMotors motor) {
    return this.motorIds[motor.getIndex()];
  }

  /**
   * Returns the motor controller for the specified motor.
   *
   * @param motor The motor.
   * @return The motor controller.
   */
  public MotorController getMotorController(SwerveMotors motor) {
    int motorID = getMotorId(motor);

    switch (motor) {
      case FrontLeftDrive:
      case FrontRightDrive:
      case BackLeftDrive:
      case BackRightDrive:
        double metersPerRotation = (getWheelDiameter() * Math.PI) / getDriveGearRatio();

        return this.driveMotor.newController(
            motorID, COUNTER_CLOCKWISE_POSITIVE, BRAKE, metersPerRotation);

      default:
        double radiansPerRotation = (2 * Math.PI) / getSteeringGearRatio();

        return this.steeringMotor.newController(
            motorID, getSteeringDirection(), BRAKE, radiansPerRotation);
    }
  }

  /**
   * Returns the CANcoder ids of the specified module.
   *
   * @param angleEncoder The angle encoder.
   * @return The CANcoder id.
   */
  public int getAngleEncoderId(SwerveAngleEncoder angleEncoder) {
    return this.angleEncoderIds[angleEncoder.getIndex()];
  }

  /**
   * Returns the angle offsets for the specified module.
   *
   * @return the angle offsets for the specified module.
   */
  public double getAngleOffset(SwerveAngleEncoder angleEncoder) {
    return this.angleOffset[angleEncoder.getIndex()];
  }

  /**
   * Returns and configures the angle encoder for the specified module.
   *
   * @param angleEncoder The angle encoder.
   * @return The angle encoder for the specified module.
   */
  public CANcoder getAngleEncoder(SwerveAngleEncoder angleEncoder) {
    int deviceID = getAngleEncoderId(angleEncoder);
    CANcoder wheelAngleEncoder = new CANcoder(deviceID);
    CANcoderConfigurator wheelAngleConfigurator = wheelAngleEncoder.getConfigurator();
    CANcoderConfiguration wheelAngleConfig = new CANcoderConfiguration();

    wheelAngleConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    wheelAngleConfig.MagnetSensor.MagnetOffset = getAngleOffset(angleEncoder) / 360.0;
    wheelAngleConfigurator.apply(wheelAngleConfig);
    return wheelAngleEncoder;
  }

  /**
   * Returns the maximum drive speed in m/s of a swerve module.
   *
   * @return The maximum drive speed.
   */
  public double getMaxDriveSpeed() {
    return this.maxDriveSpeed;
  }

  /**
   * Returns the maximum drive acceleration in m/s^2 of a swerve module.
   *
   * @return The maximum drive acceleration.
   */
  public double getMaxDriveAcceleration() {
    return this.maxDriveAcceleration;
  }

  /**
   * Returns the kS feedforward control constant for translation in Volts.
   *
   * <p>This is the voltage needed to overcome the internal friction of the motor.
   *
   * @return The kS feedforward control constant for translation in Volts.
   */
  public double getDriveKs() {
    return this.driveFeedforward.kS;
  }

  /**
   * Returns the kV feedforward control constant for translation in Volt * seconds per meter.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant velocity.
   *
   * @return The kV feedforward control constant for translation in Volt * seconds per meter.
   */
  public double getDriveKv() {
    return this.driveFeedforward.kV;
  }

  /**
   * Returns the kA feedforward control constant for translation in Volt * seconds^2 per meter.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant acceleration.
   *
   * @return The kA feedforward control constant for translation in Volt * seconds^2 per meter.
   */
  public double getDriveKa() {
    return this.driveFeedforward.kA;
  }

  /**
   * Returns a {@link SwerveDriveKinematics} object used to convert chassis speeds to individual
   * module states.
   *
   * @return A {@link SwerveDriveKinematics} object used to convert chassis speeds to individual
   *     module states.
   */
  public SwerveDriveKinematics getKinematics() {
    return this.kinematics;
  }

  /**
   * Returns a {@link SwerveDriveKinematicsConstraint} object used to enforce swerve drive
   * kinematics constraints when following a trajectory.
   *
   * @return A {@link SwerveDriveKinematicsConstraint} object used to enforce swerve drive
   *     kinematics constraints when following a trajectory.
   */
  public SwerveDriveKinematicsConstraint getKinematicsConstraint() {
    return this.kinematicsConstraint;
  }

  /**
   * Returns the maximum steering speed in rad/s of a swerve module.
   *
   * @return The maximum steering speed.
   */
  public double getMaxSteeringSpeed() {
    return this.maxSteeringSpeed;
  }

  /**
   * Returns the maximum steering acceleration in rad/s^2 of a swerve module.
   *
   * @return The maximum steering acceleration.
   */
  public double getMaxSteeringAcceleration() {
    return this.maxSteeringAcceleration;
  }

  /**
   * Returns the kS feedforward control constant for rotation in Volts.
   *
   * <p>This is the voltage needed to overcome the internal friction of the motor.
   *
   * @return The kS feedforward control constant for rotation in Volts.
   */
  public double getSteeringKs() {
    return this.steeringFeedforward.kS;
  }

  /**
   * Returns the kV feedforward control constant for rotation in Volt * seconds per radian.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant steering velocity.
   *
   * @return The kV feedforward control constant for translation in Volt * seconds per radian.
   */
  public double getSteeringKv() {
    return this.steeringFeedforward.kV;
  }

  /**
   * Returns the kA feedforward control constant for rotation in Volt * seconds^2 per radian.
   *
   * <p>This is used to calculate the voltage needed to maintain a constant rotational acceleration.
   *
   * @return The kA feedforward control constant for translation in Volt * seconds^2 per radian.
   */
  public double getSteeringKa() {
    return this.steeringFeedforward.kA;
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   * constraints on the {@link ProfiledPIDController} used to reach the goal wheel angle.
   *
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce velocity and acceleration
   *     constraints on the controller used to reach the goal wheel angle.
   */
  public TrapezoidProfile.Constraints getSteeringConstraints() {
    return steeringConstraints;
  }

  /**
   * Returns the direction the steering motor must rotate when a positive voltage is applied to
   * rotate the wheel in the counter-clockwise direction.
   *
   * @return The direction the steering motor must rotate when a positive voltage is applied to
   *     rotate the wheel in the counter-clockwise direction.
   */
  public MotorDirection getSteeringDirection() {
    return moduleParams.getSteeringDirection();
  }

  /** Returns the correct gyro implementation for the robot. */
  public Gyro getGyro() {
    if (usesPigeon) {
      return new Pigeon2Gyro(pigeonID);
    }
    return new NavXGyro();
  }

  /** Returns the moment of the robot base. */
  public double getMomentOfInertia() {
    return getRobotMass()
        * (Math.pow(getWheelDistanceX(), 2) + Math.pow(getWheelDistanceY(), 2))
        / 12;
  }

  /** Returns the robot config for pathplanner. */
  public RobotConfig getPathplannerConfig() {
    ModuleConfig moduleconfig =
        new ModuleConfig(
            getWheelDiameter() / 2,
            getMaxDriveSpeed(),
            1.0,
            driveMotor.getDCMotor(),
            getDriveGearRatio(),
            80.0,
            1);
    return new RobotConfig(getRobotMass(), getMomentOfInertia(), moduleconfig, this.wheelPositions);
  }
}
