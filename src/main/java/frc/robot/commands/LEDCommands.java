/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ColorConstants;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public class LEDCommands {

  public static Command setColor(StatusLED statusLED, Color8Bit color) {
    return Commands.runOnce(() -> statusLED.fillAndCommitColor(color), statusLED);
  }

  public static Command indicateCoralAcquired(Subsystems subsystem) {
    return Commands.sequence(
        new BlinkGreen(subsystem.statusLEDs, 1.0),
        setColor(subsystem.statusLEDs, ColorConstants.GREEN),
        Commands.idle(subsystem.statusLEDs).until(() -> !subsystem.coralRoller.hasCoral()));
  }
}
