/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public class ClimberCommands {
  /** Returns a command that climbs. */
  public static Command climb(Subsystems subsystems) {
    return Commands.runOnce(
            () -> subsystems.climber.setGoalAngle(0.0),
            subsystems.climber) // TODO: set goal angle for climber
        .withName("Climb");
  }
}
