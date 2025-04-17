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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Subsystems;

/** A namespace for algae command factory methods. */
public final class AlgaeCommands {
  /** Returns a command that removes algae at the given reef level. */
  public static Command removeAlgaeAtLevel(Subsystems subsystems, ElevatorLevel elevatorLevel) {
    Elevator elevator = subsystems.elevator;
    Arm coralArm = subsystems.coralArm;
    CoralRoller coralRoller = subsystems.coralRoller;

    return Commands.sequence(
            Commands.parallel(
                ElevatorCommands.seekToElevatorLevel(subsystems, elevatorLevel),
                CoralCommands.setArmAngleForReefLevel(subsystems, elevatorLevel),
                CoralCommands.outtakeCoral(subsystems)),
            Commands.idle(elevator, coralArm, coralRoller))
        .finallyDo(coralRoller::disable)
        .withName(String.format("RemoveAlgaeAtLevel(%s)", elevatorLevel.name()));
  }
}
