/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.PERIODIC_INTERVAL;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** A command to drive the robot on a straight line in using trapezoidal motion profiling. */
public class DriveToPose extends Command {
  private final Swerve drivetrain;
  private final HolonomicDriveController controller;
  private final Supplier<Translation2d> translationSupplier;
  private final Supplier<Rotation2d> orientationSupplier;
  private final TrapezoidProfile profile;

  private Pose2d initialPose;
  private Pose2d targetPose;

  private static final DataLog LOG = DataLogManager.getLog();

  private final StructLogEntry<Pose2d> logInitialPose =
      StructLogEntry.create(LOG, "/AlignToPose/initialPose", Pose2d.struct);
  private final StructLogEntry<Pose2d> logTargetPose =
      StructLogEntry.create(LOG, "/AlignToPose/targetPose", Pose2d.struct);
  private final StructLogEntry<Pose2d> logTrajectoryPose =
      StructLogEntry.create(LOG, "/AlignToPose/trajectoryPose", Pose2d.struct);
  private final StructLogEntry<Pose2d> logNextPose =
      StructLogEntry.create(LOG, "/AlignToPose/nextPose", Pose2d.struct);

  /**
   * Creates a new command that drives robot the specified distance in the direction of its current
   * orientation.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param distance The distance to travel in meters.
   * @param maxSpeed The maximum speed at which to travel.
   */
  public DriveToPose(Swerve drivetrain, double distance, double maxSpeed) {
    this(
        drivetrain,
        () -> new Translation2d(distance, drivetrain.getOrientation()),
        maxSpeed,
        () -> drivetrain.getOrientation());
  }

  /**
   * Creates a new command that drives robot along the specified vector at maximum speed while
   * maintaining the current orientation of the robot.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   */
  public DriveToPose(Swerve drivetrain, Translation2d translation) {
    this(
        drivetrain,
        () -> translation,
        Swerve.getMaxSpeed(),
        () -> drivetrain.getPosition().getRotation());
  }

  /**
   * Creates a new command that drives robot along the specified vector and speed while maintaining
   * the current orientation of the robot.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   */
  public DriveToPose(Swerve drivetrain, Translation2d translation, double maxSpeed) {
    this(drivetrain, () -> translation, maxSpeed, () -> drivetrain.getPosition().getRotation());
  }

  /**
   * Creates a new command that drives robot along the specified vector while rotating the robot to
   * the desired orientation.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   * @param orientation The desired orientation at the end of the command.
   */
  public DriveToPose(
      Swerve drivetrain, Translation2d translation, double maxSpeed, Rotation2d orientation) {
    this(drivetrain, () -> translation, maxSpeed, () -> orientation);
  }

  /**
   * Creates a new command that drives robot to the specified absolute location and orientation on
   * the field.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param position A {@link Pose2d} instance describing the absolute position and orientation to
   *     drive to.
   * @param maxSpeed The maximum speed at which to travel.
   */
  public DriveToPose(Swerve drivetrain, Pose2d position, double maxSpeed) {
    this(
        drivetrain,
        () -> position.getTranslation().minus(drivetrain.getPosition().getTranslation()),
        maxSpeed,
        () -> position.getRotation());
  }

  /**
   * Creates a new command that drives robot to the specified absolute location and orientation on
   * the field.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param translationSupplier Supplies a {@link Translation2d} instance describing the line on
   *     which to travel. This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   * @param orientationSupplier Supplies the desired orientation at the end of the command.
   */
  private DriveToPose(
      Swerve drivetrain,
      Supplier<Translation2d> translationSupplier,
      double maxSpeed,
      Supplier<Rotation2d> orientationSupplier) {
    this.drivetrain = drivetrain;
    this.translationSupplier = translationSupplier;
    this.controller = drivetrain.createDriveController();
    this.profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxSpeed, Swerve.getMaxAcceleration() * 0.3));
    this.orientationSupplier = orientationSupplier;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    initialPose = drivetrain.getPosition();
    Translation2d translation = translationSupplier.get();
    Rotation2d orientation = orientationSupplier.get();

    targetPose = initialPose.plus(new Transform2d(translation, orientation));

    logInitialPose.append(initialPose);
    logTargetPose.append(targetPose);
  }

  @Override
  public void execute() {
    var currentPose = drivetrain.getPosition();
    var currentSpeeds = drivetrain.getChassisSpeeds();
    var delta = targetPose.getTranslation().minus(currentPose.getTranslation());
    double distanceToGo = delta.getNorm();
    Rotation2d heading = delta.getAngle();

    // Calculate the current speed along the desired heading.
    double currentSpeed =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
            * Math.cos(currentPose.getRotation().minus(heading).getRadians());

    // Calculate the next setpoint of motion (position and velocity) using the
    // trapezoidal profile.
    var currentState = new TrapezoidProfile.State(0, currentSpeed);
    var goalState = new TrapezoidProfile.State(distanceToGo, 0);
    var setpoint = profile.calculate(PERIODIC_INTERVAL, currentState, goalState);

    // Determine the next position on the field by offsetting the initial position
    // by the distance moved along the line of travel.
    Translation2d offset = new Translation2d(setpoint.position, heading);
    Pose2d trajectoryPose = new Pose2d(currentPose.getTranslation().plus(offset), heading);

    // Calculate the swerve drive module states needed to reach the next state.
    ChassisSpeeds speeds =
        controller.calculate(
            drivetrain.getPosition(), trajectoryPose, setpoint.velocity, targetPose.getRotation());

    drivetrain.setChassisSpeeds(speeds);

    var nextPose =
        new Pose2d(
            trajectoryPose.getTranslation(),
            currentPose
                .getRotation()
                .plus(new Rotation2d(speeds.omegaRadiansPerSecond * PERIODIC_INTERVAL)));

    logTrajectoryPose.append(trajectoryPose);
    logNextPose.append(nextPose);
  }

  @Override
  public boolean isFinished() {
    return profile.isFinished(0);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }
}
