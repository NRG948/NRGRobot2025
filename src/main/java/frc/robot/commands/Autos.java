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

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command none(Subsystems subsystem) {
    return Commands.none();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
