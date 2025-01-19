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
public class CoralCommands {
  /** Returns a command that intakes coral. */
  public static Command intakeCoral(Subsystems subsystems) {
    return Commands.none();
  }

  /** Returns a command that outtakes coral. */
  public static Command outtakeCoral(Subsystems subsystems) {
    return Commands.none();
  }

  /**
   * Returns the command sequence that outtakes the coral until not detected and then stows the
   * elevator and arm.
   *
   * @param subsystems
   * @return
   */
  public static Command deliverCoral(Subsystems subsystems) {
    return Commands.none();
  }
}
