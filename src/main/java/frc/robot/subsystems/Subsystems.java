/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class Subsystems {
  public final SwerveSubsystem drivetrain = new SwerveSubsystem();
  public final Elevator elevator = new Elevator();
  public final CoralArm coralArm = new CoralArm();
  public final Optional<AprilTagSubsystem> aprilTag =
      newOptionalSubsystem(
          AprilTagSubsystem.class,
          AprilTagSubsystem.ENABLED,
          Constants.CAMERA1_NAME,
          Constants.APRILTAG_ROBOT_TO_CAMERA1);

  ArrayList<Subsystem> all =
      new ArrayList<Subsystem>(Arrays.asList(drivetrain, elevator, coralArm));
  ArrayList<Subsystem> manipulators = new ArrayList<Subsystem>(Arrays.asList(elevator, coralArm));

  /** Constructs the robot subsystems container. */
  public Subsystems() {
    if (aprilTag.isPresent()) {
      all.add(aprilTag.get());
    }
  }

  /** Returns an array of all subsystems. */
  public Subsystem[] getAll() {
    return all.toArray(Subsystem[]::new);
  }

  /** Returns an array of all manipulator subsystems. */
  public Subsystem[] getManipulators() {
    return manipulators.toArray(Subsystem[]::new);
  }

  /**
   * Creates a new optional subsystem.
   *
   * @param <T> The of subsystem.
   * @param subsystemClass The subsystem class.
   * @param enabled The preferences value indicating whether the subsystem is enabled.
   * @param initArgs The arguments to pass to the subsystem's constructor.
   * @return Returns a non-empty {@link Optional} instance if the subsystem is enabled. Otherwise,
   *     this method returns {@link Optional#empty}.
   */
  private static <T extends Subsystem> Optional<T> newOptionalSubsystem(
      Class<T> subsystemClass, RobotPreferences.BooleanValue enabled, Object... initArgs) {
    if (!enabled.getValue()) {
      return Optional.empty();
    }

    Class<?>[] initArgClasses = Stream.of(initArgs).map(Object::getClass).toArray(Class<?>[]::new);

    try {
      return Optional.of(subsystemClass.getConstructor(initArgClasses).newInstance(initArgs));
    } catch (InstantiationException
        | IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException
        | SecurityException e) {
      System.err.printf(
          "ERROR: An unexpected exception was caught while creating an instance of %s.%n",
          subsystemClass.getName());
      e.printStackTrace();
      return Optional.empty();
    } catch (NoSuchMethodException e) {
      System.err.printf(
          "ERROR: The class is missing constructor %s(%s).%n",
          subsystemClass.getName(),
          Stream.of(initArgClasses).map(Class::getSimpleName).collect(Collectors.joining(", ")));
      e.printStackTrace();
      return Optional.empty();
    }
  }

  public void initShuffleboard() {
    for (Subsystem subsystem : all) {
      if (subsystem instanceof ShuffleboardProducer) {
        ShuffleboardProducer.class.cast(subsystem).addShuffleboardTab();
      }
    }
  }

  public void periodic() {
    if (aprilTag.isPresent()) {
      AprilTagSubsystem aprilTag = this.aprilTag.get();
      var visionEst = aprilTag.getEstimateGlobalPose();

      visionEst.ifPresent(
          (est) -> {
            var estPose = est.estimatedPose.toPose2d();
            var estStdDevs = aprilTag.getEstimationStdDevs();

            drivetrain.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
          });
    }
  }
}
