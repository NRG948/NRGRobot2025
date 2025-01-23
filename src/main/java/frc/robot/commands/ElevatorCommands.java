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
  public static Command goToElevatorLevel(Subsystems subsystems, ElevatorLevel level) {
    return Commands.runOnce(() -> subsystems.elevator.setGoalPosition(level), subsystems.elevator);
  }

  /** Returns a command that waits for elevator to reach goal position. */
  public static Command waitForElevatorToReachGoalPosition(Subsystems subsystems) {
    return Commands.idle(subsystems.elevator).until(subsystems.elevator::atGoalPosition);
  }

  /** Returns a command that stows elevator. */
  public static Command stowElevator(Subsystems subsystems) {
    return Commands.sequence(
        Commands.runOnce(
            () -> subsystems.elevator.setGoalPosition(ElevatorLevel.STOWED), subsystems.elevator),
        waitForElevatorToReachGoalPosition(subsystems),
        Commands.runOnce(() -> subsystems.elevator.disable(), subsystems.elevator));
  }

  /** Returns a command that stows the elevator and the arm. */
  public static Command stowElevatorAndArm(Subsystems subsystems) {
    return Commands.parallel(stowElevator(subsystems), CoralCommands.stowArm(subsystems));
  }
}
