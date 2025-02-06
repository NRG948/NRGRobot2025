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

/** A namespace for manipulator command factory methods. */
public final class ManipulatorCommands {
  /** Returns a command that interrupts all manipulator subsystems. */
  public static Command interruptAll(Subsystems subsystems) {
    return Commands.runOnce(() -> {}, subsystems.getManipulators())
        .withName("InterruptManipulators");
  }
}
