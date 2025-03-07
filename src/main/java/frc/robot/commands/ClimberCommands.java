/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;

/** A namespace for climber command factory methods. */
public final class ClimberCommands {
  private static final double CLIMB_ANGLE = Math.toRadians(-93);

  /** Returns a command that climbs. */
  public static Command climb(Subsystems subsystems) {
    Climber climber = subsystems.climber;
    StatusLED statusLEDs = subsystems.statusLEDs;

    return Commands.parallel(
            Commands.runOnce(() -> climber.setGoalAngle(CLIMB_ANGLE)), //
            new RainbowCycle(statusLEDs))
        .withName("Climb");
  }
}
