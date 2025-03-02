/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.AprilTag;
import java.util.Optional;

public final class VisionCommands {
  private VisionCommands() {
    // Prevent instantiation
    throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
  }

  /**
   * Returns a command that waits for the AprilTag camera to detect a target.
   *
   * @param optionalCamera The AprilTag camera subsystem. This may be `Optional.empty()` if the
   *     camera is not present.
   * @param idleSubsystem The subsystem to idle while waiting.
   * @return A command that waits for the AprilTag camera to detect a target.
   */
  public static Command waitForAprilTag(
      Optional<AprilTag> optionalCamera, Subsystem idleSubsystem) {
    return optionalCamera
        .map((camera) -> (Command) Commands.idle(idleSubsystem).until(camera::hasTargets))
        .orElse(Commands.none());
  }
}
