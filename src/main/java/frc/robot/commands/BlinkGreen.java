/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

/** A command to blink the status LEDs green. */
public class BlinkGreen extends Command {
  private static final double BLINK_TIME = 0.2;

  private final LEDSubsystem led;
  private final double duration;
  private final Timer timer = new Timer();
  private boolean isGreen;

  /** Creates a new BlinkGreen. */
  public BlinkGreen(LEDSubsystem led, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.duration = duration;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.fillAndCommitColor(Constants.ColorConstants.GREEN);
    isGreen = true;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (timer.advanceIfElapsed(BLINK_TIME)) {
      if (isGreen) {
        led.fillAndCommitColor(Constants.ColorConstants.BLACK);
      } else {
        led.fillAndCommitColor(Constants.ColorConstants.GREEN);
      }
      isGreen = !isGreen;
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
