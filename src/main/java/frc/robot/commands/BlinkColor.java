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
  // Using 0.2 results in the LEDs to end up as no color
  private static final double BLINK_TIME = 0.19;

  private final LEDSubsystem led;
  private final Colors color;
  private final double duration;
  private final Timer blinkTimer = new Timer();
  private final Timer endTimer = new Timer();
  private boolean isOn;

  /** Creates a new BlinkColor. */
  public BlinkColor(LEDSubsystem led, Colors color, double duration) {
    setName(String.format("BlinkColor(%s)", color.name()));

    this.led = led;
    this.color = color;
    this.duration = duration;

    addRequirements(led);
  }

  /** Creates a new BlinkColor that runs until interrupted. */
  public BlinkColor(LEDSubsystem led, Colors color) {
    this(led, color, Double.MAX_VALUE);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.fillAndCommitColor(color);
    isOn = true;
    blinkTimer.reset();
    blinkTimer.start();
    endTimer.reset();
    endTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (blinkTimer.advanceIfElapsed(BLINK_TIME)) {
      led.fillAndCommitColor(isOn ? BLACK : color);
      isOn = !isOn;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    blinkTimer.stop();
    endTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return endTimer.hasElapsed(duration);
  }
}
