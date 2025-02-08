/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.Colors.BLACK;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.parameters.Colors;
import frc.robot.subsystems.LEDSubsystem;

/** A command to blink the status LEDs a specified color. */
public final class BlinkColor extends Command {
  private static final double BLINK_TIME = 0.2;

  private final LEDSubsystem led;
  private final Colors color;
  private final double duration;
  private final Timer timer = new Timer();
  private boolean isOn;

  /** Creates a new BlinkColor. */
  public BlinkColor(LEDSubsystem led, Colors color, double duration) {
    setName(String.format("BlinkColor(%s)", color.name()));

    this.led = led;
    this.color = color;
    this.duration = duration;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.fillAndCommitColor(color);
    isOn = true;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (timer.advanceIfElapsed(BLINK_TIME)) {
      if (isOn) {
        led.fillAndCommitColor(BLACK);
      } else {
        led.fillAndCommitColor(color);
      }
      isOn = !isOn;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(duration);
  }
}
