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
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Subsystems;

/** A namespace for algae command factory methods. */
public final class AlgaeCommands {
  private static final double STOWED_ANGLE = Math.toRadians(90.0);

  /** Returns a command to intake algae. */
  public static Command intakeAlgae(Subsystems subsystems) {
    return subsystems
        .algaeGrabber
        .map(
            (algaeGrabber) ->
                (Command)
                    Commands.runOnce(() -> algaeGrabber.intake(), algaeGrabber)
                        .withName("IntakeAlgae"))
        .orElse(Commands.none());
  }

  /** Returns a command to outtake algae. */
  public static Command outtakeAlgae(Subsystems subsystems) {
    return subsystems
        .algaeGrabber
        .map(
            (algaeGrabber) ->
                (Command)
                    Commands.runOnce(() -> algaeGrabber.outtake(), algaeGrabber)
                        .withName("OuttakeAlgae"))
        .orElse(Commands.none());
  }

  /** Returns a command to stop the algae grabber. */
  public static Command stopAlgaeGrabber(Subsystems subsystems) {
    return subsystems
        .algaeGrabber
        .map(
            (algaeGrabber) ->
                (Command)
                    Commands.runOnce(() -> algaeGrabber.disable(), algaeGrabber)
                        .withName("StopAlgaeGrabber"))
        .orElse(Commands.none());
  }

  /** Returns a command that removes algae at the given reef level. */
  public static Command removeAlgaeAtLevel(Subsystems subsystems, ElevatorLevel elevatorLevel) {
    Elevator elevator = subsystems.elevator;
    Arm coralArm = subsystems.coralArm;
    CoralRoller coralRoller = subsystems.coralRoller;

    return Commands.sequence(
            Commands.parallel(
                ElevatorCommands.goToElevatorLevel(subsystems, elevatorLevel),
                CoralCommands.setArmAngleForReefLevel(subsystems, elevatorLevel),
                CoralCommands.outtakeCoral(subsystems)),
            Commands.idle(elevator, coralArm, coralRoller))
        .finallyDo(coralRoller::disable)
        .withName(String.format("RemoveAlgaeAtLevel(%s)", elevatorLevel.name()));
  }

  /** Returns a command that stops and stows the intake. */
  public static Command stopAndStowIntake(Subsystems subsystems) {
    if (subsystems.algaeGrabber.isEmpty() || subsystems.algaeArm.isEmpty()) {
      return Commands.none();
    }

    AlgaeGrabber algaeGrabber = subsystems.algaeGrabber.get();
    Arm algaeArm = subsystems.algaeArm.get();

    return Commands.parallel(
            Commands.runOnce(() -> algaeGrabber.disable(), algaeGrabber),
            Commands.runOnce(() -> algaeArm.setGoalAngle(STOWED_ANGLE), algaeArm))
        .withName("StopAndStowIntake");
  }
}
