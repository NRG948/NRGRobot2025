/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AlignToReef.ReefPosition;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Subsystems;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionCommands {
  /**
   * Returns a Command Seuquence to allignToTheReef and Score coral in the given reefPosition and Elevator Level
   * @param subsystems The Subsystems Container
   * @param reefPosition The specified Reef Position
   * @param level The specified Eleator level
   * @return A Command Seuquence to allignToTheReef and Score coral in the given reefPosition and Elevator Level
   */
  public static Command autoScoreCoral(Subsystems subsystems, ReefPosition reefPosition, ElevatorLevel level) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.alignToReefPosition(subsystems, reefPosition),
            CoralAndElevatorCommands.raiseElevatorAndCoralArm(subsystems, level)),
        CoralCommands.outtakeUntilCoralNotDetected(subsystems),
        ElevatorCommands.stowElevatorAndArmForCoral(subsystems));
  }

  public static Command autoIntakeToCoralStation(Subsystems subsystems) {
    return Commands.sequence(
        DriveCommands.alignToCoralStationCenter(subsystems),
        CoralCommands.intakeUntilCoralDetected(subsystems));
  }
}
