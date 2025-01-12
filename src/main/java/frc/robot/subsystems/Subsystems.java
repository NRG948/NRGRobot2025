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

public class Subsystems {
  public final SwerveSubsystem drivetrain = new SwerveSubsystem();
  public final Elevator elevator = new Elevator();
  public final Optional<AprilTagSubsystem> aprilTag;

  // public final Subsystem[] all = new Subsystem[] {drivetrain, elevator};

  ArrayList<Subsystem> all = new ArrayList<Subsystem>(Arrays.asList(drivetrain, elevator));

  public Subsystems() {
    if (AprilTagSubsystem.ENABLED.getValue()) {
      aprilTag =
          Optional.of(
              new AprilTagSubsystem(Constants.CAMERA1_NAME, Constants.APRILTAG_ROBOT_TO_CAMERA1));
      all.add(aprilTag.get());
    } else {
      aprilTag = Optional.empty();
    }
  }

  /**
   * Creates a new optional subsystem.
   *
   * @param <T> The of subsystem.
   * @param subsystemClass The subsystem class.
   * @param enabled The preferences value indicating whether the subsystem is enabled.
   * @return Returns a non-empty {@link Optional} instance if the subsystem is enabled. Otherwise,
   *     this method returns {@link Optional#empty}.
   */
  private static <T extends Subsystem> Optional<T> newOptionalSubsystem(
      Class<T> subsystemClass, RobotPreferences.BooleanValue enabled) {

    if (!enabled.getValue()) {
      return Optional.empty();
    }

    try {
      return Optional.of(subsystemClass.getConstructor().newInstance());
    } catch (InstantiationException
        | IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException
        | NoSuchMethodException
        | SecurityException e) {
      System.err.printf(
          "ERROR: An unexpected exception was caught while creating an instance of %s.%n",
          subsystemClass.getName());
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
}
