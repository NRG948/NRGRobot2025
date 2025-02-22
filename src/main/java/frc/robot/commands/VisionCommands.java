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
   * Returns a command to use AllignToReef to allign to a reef position and score coral at Level 1.
   *
   * @param subsystems The Subsystems Container.
   * @param reefPosition The Specified reef position.
   * @return
   */
  public static Command AutoScoreToL1(Subsystems subsystems, ReefPosition reefPosition) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.alignToReefPosition(subsystems, reefPosition),
            CoralAndElevatorCommands.raiseElevatorAndCoralArm(subsystems, ElevatorLevel.L1)),
        CoralCommands.outtakeUntilCoralNotDetected(subsystems),
        ElevatorCommands.stowElevatorAndArmForCoral(subsystems));
  }

  /**
   * Returns a command to use AllignToReef to allign to a reef position and score coral at Level 2.
   *
   * @param subsystems Subsystems container.
   * @param reefPosition Specified reef Position.
   * @return
   */
  public static Command AutoScoreToL2(Subsystems subsystems, ReefPosition reefPosition) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.alignToReefPosition(subsystems, reefPosition),
            CoralAndElevatorCommands.raiseElevatorAndCoralArm(subsystems, ElevatorLevel.L2)),
        CoralCommands.outtakeUntilCoralNotDetected(subsystems),
        ElevatorCommands.stowElevatorAndArmForCoral(subsystems));
  }

  /**
   * Returns a command to use AllignToReef to allign to a reef position and score coral at Level 3.
   *
   * @param subsystems The Subsystems Container.
   * @param reefPosition Specified Reef Position.
   * @return
   */
  public static Command AutoScoreToL3(Subsystems subsystems, ReefPosition reefPosition) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.alignToReefPosition(subsystems, reefPosition),
            CoralAndElevatorCommands.raiseElevatorAndCoralArm(subsystems, ElevatorLevel.L3)),
        CoralCommands.outtakeUntilCoralNotDetected(subsystems),
        ElevatorCommands.stowElevatorAndArmForCoral(subsystems));
  }

  /**
   * Returns a command to use AllignToReef to allign to a reef position and score coral at Level 4.
   *
   * @param subsystems Subsystems Container.
   * @param reefPosition Specified Reef Position.
   * @return
   */
  public static Command AutoScoreToL4(Subsystems subsystems, ReefPosition reefPosition) {
    return Commands.sequence(
        Commands.parallel(
            DriveCommands.alignToReefPosition(subsystems, reefPosition),
            CoralAndElevatorCommands.raiseElevatorAndCoralArm(subsystems, ElevatorLevel.L4)),
        CoralCommands.outtakeUntilCoralNotDetected(subsystems),
        ElevatorCommands.stowElevatorAndArmForCoral(subsystems));
  }

  public static Command AutoIntakeToCoralStation(Subsystems subsystems) {
    return Commands.sequence(
        DriveCommands.alignToCoralStationCenter(subsystems),
        CoralCommands.intakeUntilCoralDetected(subsystems));
  }
}
