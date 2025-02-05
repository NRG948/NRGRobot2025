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

public class BlinkGreen extends Command {

  private final LEDSubsystem led;
  private final Timer timer = new Timer();
  private boolean isGreen;
  private final double duration;

  /** Creates a new BlinkingGreen. */
  public BlinkGreen(LEDSubsystem led, double duration) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.duration = duration;
    this.led = led;
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

    if (timer.advanceIfElapsed(0.2)) {
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
