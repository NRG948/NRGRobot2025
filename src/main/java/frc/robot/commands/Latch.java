/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.ElevatorLevel.STOWED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;

public class Latch extends Command {

  private final Climber climber;
  private final StatusLED statusLEDs;
  private final Arm coralArm;

  public Latch(Subsystems subsystems) {
    this.climber = subsystems.climber;
    this.statusLEDs = subsystems.statusLEDs;
    this.coralArm = subsystems.coralArm;

    addRequirements(climber, statusLEDs, coralArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralArm.setGoalAngle(STOWED.getArmAngle());
    climber.setGoalAngle(ClimberCommands.CLIMB_ANGLE);
    new RainbowCycle(statusLEDs);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.withinLowKPRange()) {
      climber.enterLowKPMode();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.exitLowKPMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
