/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Subsystems;

public final class DriveCommands {
  /**
   * Returns a command that resets the orientation of the drivetrain.
   *
   * @param subsystems The subsystems container.
   * @return
   */
  public static Command resetOrientation(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.drivetrain.resetOrientation(new Rotation2d()));
  }

  public static Command alignToLeftBranch(Subsystems subsystems) {
    return Commands.none();
  }

  public static Command alignToRightBranch(Subsystems subsystems) {
    return Commands.none();
  }

  public static Command inputScalarDrivetrain(Subsystems subsystems) {
    return Commands.none();
  }

  public static Command interruptAll(Subsystems subsystems) {
    return Commands.runOnce(() -> {}, subsystems.getAll());
  }
}
