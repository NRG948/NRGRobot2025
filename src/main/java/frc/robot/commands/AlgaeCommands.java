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

/** Add your docs here. */
public class AlgaeCommands {
  /** Returns a command to intake algae. */
  public static Command intakeAlgae(Subsystems subsystems) {
    return Commands.none();
  }

  /** Returns a command to outtake algae. */
  public static Command outtakeAlgae(Subsystems subsystems) {
    return Commands.none();
  }

  /** Returns a command that removes algae at the given reef level. */
  public static Command removeAlgaeAtLevel(Subsystems subsystems, ElevatorLevel elevatorLevel) {
    return Commands.none();
  }

  /** Returns a command that stops and stows the intake. */
  public static Command stopAndStowIntake(Subsystems subsystems) {
    return Commands.none();
  }
}
