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

public final class CoralAndElevatorCommands {
  /** Raises elevator and coral arm based on pivot height. */
  public static Command raiseElevatorAndTipCoralArm(Subsystems subsystems, ElevatorLevel level) {
    return Commands.parallel(
            ElevatorCommands.goToElevatorLevel(subsystems, level),
            Commands.sequence(
                CoralCommands.waitForElevatorToReachArmHeight(subsystems),
                CoralCommands.setArmAngleForReefLevel(subsystems, level)))
        .withName("RaiseElevatorAndCoralArm");
  }
}
