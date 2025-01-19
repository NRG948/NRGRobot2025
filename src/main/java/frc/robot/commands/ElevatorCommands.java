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
public class ElevatorCommands {

  /** Returns a command that goes to the given elevator level. */
  public static Command goToElevatorLevel(Subsystems subsystems, ElevatorLevel elevatorLevel) {
    return Commands.none();
  }

  /** Returns a command that stows the elevator and the arm. */
  public static Command stowElevatorAndArm(Subsystems subsystems) {
    return Commands.none();
  }
}
