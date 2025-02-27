/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Subsystems;

/** A namespace for algae command factory methods. */
public final class AlgaeCommands {
  private static final double STOWED_ANGLE = Math.toRadians(90.0);

  /** Returns a command to intake algae. */
  public static Command intakeAlgae(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.algaeGrabber.intake(), subsystems.algaeGrabber)
        .withName("IntakeAlgae");
  }

  /** Returns a command to outtake algae. */
  public static Command outtakeAlgae(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.algaeGrabber.outtake(), subsystems.algaeGrabber)
        .withName("OuttakeAlgae");
  }

  /** Returns a command to stop the algae grabber. */
  public static Command stopAlgaeGrabber(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.algaeGrabber.disable(), subsystems.algaeGrabber)
        .withName("StopAlgaeGrabber");
  }

  /** Returns a command that removes algae at the given reef level. */
  public static Command removeAlgaeAtLevel(Subsystems subsystems, ElevatorLevel elevatorLevel) {
    return Commands.sequence(
            Commands.parallel(
                ElevatorCommands.goToElevatorLevel(subsystems, elevatorLevel),
                CoralCommands.setArmAngleForReefLevel(subsystems, elevatorLevel),
                CoralCommands.outtakeCoral(subsystems)),
            Commands.idle(subsystems.elevator, subsystems.coralArm, subsystems.coralRoller))
        .finallyDo(subsystems.coralRoller::disable)
        .withName(String.format("RemoveAlgaeAtLevel(%s)", elevatorLevel.name()));
  }

  /** Returns a command that stops and stows the intake. */
  public static Command stopAndStowIntake(Subsystems subsystems) {
    return Commands.parallel(
            Commands.runOnce(() -> subsystems.algaeGrabber.disable(), subsystems.algaeGrabber),
            Commands.runOnce(
                () -> subsystems.algaeArm.setGoalAngle(STOWED_ANGLE), subsystems.algaeArm))
        .withName("StopAndStowIntake");
  }
}
