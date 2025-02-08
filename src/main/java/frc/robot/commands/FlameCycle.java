/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.Colors.BLACK;
import static frc.robot.parameters.Colors.RED;
import static frc.robot.parameters.Colors.YELLOW;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StatusLED;

/** A command to display an animated flame pattern on the status LEDs. */
public final class FlameCycle extends Command {
  private static final int RED_DIFF = RED.getRed() - YELLOW.getRed();
  private static final int GREEN_DIFF = RED.getGreen() - YELLOW.getGreen();
  private static final int BLUE_DIFF = RED.getBlue() - YELLOW.getBlue();

  private final StatusLED led;
  private final int ledCount;

  private int step;

  /** Creates a new FlameCycle. */
  public FlameCycle(StatusLED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.ledCount = led.getLEDCount();
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
    led.fillColor(BLACK);
    led.commitColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double multiplier = Math.sin(step++ * Math.toRadians(6)) * 0.5 + 0.5;
    Color8Bit color0 =
        new Color8Bit(
            (int) (-(RED_DIFF * multiplier) + RED.getRed()),
            (int) (-(GREEN_DIFF * multiplier) + RED.getGreen()),
            (int) (-(BLUE_DIFF * multiplier) + RED.getBlue()));
    Color8Bit color1 =
        new Color8Bit(
            (int) (RED_DIFF * multiplier + YELLOW.getRed()),
            (int) (GREEN_DIFF * multiplier + YELLOW.getGreen()),
            (int) (BLUE_DIFF * multiplier + YELLOW.getBlue()));
    for (int i = 0; i < ledCount; i++) {
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
