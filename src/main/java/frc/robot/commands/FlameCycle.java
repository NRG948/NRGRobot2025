/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.StatusLED;

public class FlameCycle extends Command {
  private final StatusLED led;
  private final int redDiff = ColorConstants.RED.red - ColorConstants.YELLOW.red;
  private final int greenDiff = ColorConstants.RED.green - ColorConstants.YELLOW.green;
  private final int blueDiff = ColorConstants.RED.blue - ColorConstants.YELLOW.blue;

  private int step;

  /** Creates a new FlameCycle. */
  public FlameCycle(StatusLED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
    led.fillColor(ColorConstants.BLACK);
    led.commitColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double multiplier = Math.sin(step++ * Math.toRadians(6)) * 0.5 + .5;
    Color8Bit color0 =
        new Color8Bit(
            (int) (-(redDiff * multiplier) + ColorConstants.RED.red),
            (int) (-(greenDiff * multiplier) + ColorConstants.RED.green),
            (int) (-(blueDiff * multiplier) + ColorConstants.RED.blue));
    Color8Bit color1 =
        new Color8Bit(
            (int) (redDiff * multiplier + ColorConstants.YELLOW.red),
            (int) (greenDiff * multiplier + ColorConstants.YELLOW.green),
            (int) (blueDiff * multiplier + ColorConstants.YELLOW.blue));
    for (int i = 0; i < RobotConstants.LED_COUNT; i++) {
      Color8Bit color = ((i / 3) % 2) == 0 ? color0 : color1;
      led.setColor(color, i);
    }
    led.commitColor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
