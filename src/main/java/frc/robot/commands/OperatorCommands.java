/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** A namespace for command factory methods. */
public final class OperatorCommands {
  /**
   * Returns a command to rumble the Xbox controller for specified duration.
   *
   * @param controller Xbox controller
   * @param rumbleType Indicates rumbling for left, right, or both sides of controller
   * @param duration Duration of rumble in seconds
   * @return
   */
  public static Command rumbleController(
      CommandXboxController controller, RumbleType rumbleType, double duration) {
    XboxController hid = controller.getHID();

    return Commands.sequence(
        Commands.runOnce(() -> hid.setRumble(rumbleType, 1.0)),
        Commands.waitSeconds(duration),
        Commands.runOnce(() -> hid.setRumble(rumbleType, 0.0)));
  }
}
