/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** A command to drive the robot on a straight line in using trapezoidal motion profiling. */
public class DriveStraight extends Command {
  private final Swerve drivetrain;
  private final HolonomicDriveController controller;
  private final Supplier<Translation2d> translationSupplier;
  private final double maxSpeed;
  private final Supplier<Rotation2d> orientationSupplier;
  private final Timer timer = new Timer();
  private Pose2d initialPose;
  private Rotation2d heading;
  private Rotation2d orientation;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State initialState;
  private TrapezoidProfile.State goalState;

  private static final DataLog LOG = DataLogManager.getLog();

  private final DoubleLogEntry logOrientation =
      new DoubleLogEntry(LOG, "DriveStraight/orientation");
  private final DoubleLogEntry logDistance = new DoubleLogEntry(LOG, "DriveStraight/distance");
  private final DoubleLogEntry logHeading = new DoubleLogEntry(LOG, "DriveStraight/heading");
  private final DoubleLogEntry logInitialPoseX =
      new DoubleLogEntry(LOG, "DriveStraight/initialPoseX");
  private final DoubleLogEntry logInitialPoseY =
      new DoubleLogEntry(LOG, "DriveStraight/initialPoseY");
  private final DoubleLogEntry logFinalPoseX = new DoubleLogEntry(LOG, "DriveStraight/finalPoseX");
  private final DoubleLogEntry logFinalPoseY = new DoubleLogEntry(LOG, "DriveStraight/finalPoseY");

  /**
   * Creates a new DriveStraight that drives robot along the specified vector at maximum speed while
   * maintaining the current orientation of the robot.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   */
  public DriveStraight(Swerve drivetrain, Translation2d translation) {
    this(
        drivetrain,
        () -> translation,
        Swerve.getMaxSpeed(),
        () -> drivetrain.getPosition().getRotation());
  }

  /**
   * Creates a new DriveStraight that drives robot along the specified vector and speed while
   * maintaining the current orientation of the robot.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   */
  public DriveStraight(Swerve drivetrain, Translation2d translation, double maxSpeed) {
    this(drivetrain, () -> translation, maxSpeed, () -> drivetrain.getPosition().getRotation());
  }

  /**
   * Creates a new DriveStraight that drives robot along the specified vector while rotating the
   * robot to the desired orientation.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on which to travel.
   *     This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   * @param orientation The desired orientation at the end of the command.
   */
  public DriveStraight(
      Swerve drivetrain, Translation2d translation, double maxSpeed, Rotation2d orientation) {
    this(drivetrain, () -> translation, maxSpeed, () -> orientation);
  }

  /**
   * Creates a new DriveStraight that drives robot to the specified absolute location and
   * orientation on the field.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param position A {@link Pose2d} instance describing the absolute position and orientation to
   *     drive to.
   * @param maxSpeed The maximum speed at which to travel.
   */
  public DriveStraight(Swerve drivetrain, Pose2d position, double maxSpeed) {
    this(
        drivetrain,
        () -> position.getTranslation().minus(drivetrain.getPosition().getTranslation()),
        maxSpeed,
        () -> position.getRotation());
  }

  public DriveStraight(Swerve drivetrain, double distance, double maxSpeed) {
    this(drivetrain, new Translation2d(distance, 0), maxSpeed, drivetrain.getOrientation());
  }

  /**
   * Creates a new DriveStraight that drives robot to the specified absolute location and
   * orientation on the field.
   *
   * @param drivetrain The {@link Swerve} representing the robot drivetrain.
   * @param translationSupplier Supplies a {@link Translation2d} instance describing the line on
   *     which to travel. This is a vector relative to the current position.
   * @param maxSpeed The maximum speed at which to travel.
   * @param orientationSupplier Supplies the desired orientation at the end of the command.
   */
  private DriveStraight(
      Swerve drivetrain,
      Supplier<Translation2d> translationSupplier,
      double maxSpeed,
      Supplier<Rotation2d> orientationSupplier) {
    this.drivetrain = drivetrain;
    this.translationSupplier = translationSupplier;
    this.controller = drivetrain.createDriveController();
    this.maxSpeed = maxSpeed;
    this.orientationSupplier = orientationSupplier;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    initialPose = drivetrain.getPosition();
    Translation2d translation = translationSupplier.get();
    double distance = translation.getNorm();
    heading = translation.getAngle();
    orientation = orientationSupplier.get();
    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxSpeed, Swerve.getMaxAcceleration() * 0.3));

    initialState = new TrapezoidProfile.State(0, 0);
    goalState = new TrapezoidProfile.State(distance, 0);

    logOrientation.append(orientation.getDegrees());
    logDistance.append(distance);
    logHeading.append(heading.getDegrees());
    logInitialPoseX.append(initialPose.getX());
    logInitialPoseY.append(initialPose.getY());

    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Calculate the next state (position and velocity) of motion using the
    // trapezoidal profile.
    TrapezoidProfile.State state = profile.calculate(timer.get(), initialState, goalState);

    // Determine the next position on the field by offsetting the initial position
    // by the distance moved along the line of travel.
    Translation2d offset = new Translation2d(state.position, heading);
    Pose2d nextPose = new Pose2d(initialPose.getTranslation().plus(offset), heading);

    // Calculate the swerve drive module states needed to reach the next state.
    ChassisSpeeds speeds =
        controller.calculate(drivetrain.getPosition(), nextPose, state.velocity, orientation);

    drivetrain.setChassisSpeeds(speeds);
  }

  @Override
  public boolean isFinished() {
    return profile.isFinished(timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
    timer.stop();
    Pose2d finalPose = drivetrain.getPosition();
    logFinalPoseX.append(finalPose.getX());
    logFinalPoseY.append(finalPose.getY());
    logOrientation.append(finalPose.getRotation().getDegrees());
  }
}
