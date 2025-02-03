/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.StatusLEDSubsystem;

public class RainbowCycle extends Command {
  private final StatusLEDSubsystem led;
  private int step;

  /** Creates a new RainbowCycle. */
  public RainbowCycle(StatusLEDSubsystem led) {
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
    int firstPixelHue = step++ * 3;
    for (int i = 0; i < RobotConstants.LED_COUNT; i++) {
      Color8Bit color =
          new Color8Bit(Color.fromHSV((firstPixelHue + ((180 * (i + 1)) / 21)) % 180, 255, 255));
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
