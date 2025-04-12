/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.ElevatorLevel.STOWED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;

/** A namespace for climber command factory methods. */
public final class ClimberCommands {
  public static final double STOW_ANGLE = Math.toRadians(90);
  public static final double GROUND_INTAKE_ANGLE_FOR_CLIMBING = Math.toRadians(97);

  /** Returns a command that performs a Reefscape deep climb. */
  public static Command climb(Subsystems subsystems) {
    Climber climber = subsystems.climber;
    StatusLED statusLEDs = subsystems.statusLEDs;
    Arm coralArm = subsystems.coralArm;

    return Commands.parallel(
            Commands.runOnce(() -> climber.setDeepClimbAngle(), climber),
            Commands.runOnce(() -> coralArm.setGoalAngle(STOWED.getArmAngle()), coralArm),
            new RainbowCycle(statusLEDs))
        .withName("Climb");
  }

  public static Command unclimb(Subsystems subsystems) {
    return Commands.sequence(
            Commands.runOnce(() -> subsystems.climber.setGoalAngle(STOW_ANGLE), subsystems.climber),
            Commands.idle(subsystems.climber).until(() -> subsystems.climber.atGoalAngle()))
        .finallyDo(subsystems.climber::disable);
  }

  public static Command prepareToClimb(Subsystems subsystems) {
    return Commands.runOnce(
        () -> subsystems.coralIntakeArm.setGoalAngle(GROUND_INTAKE_ANGLE_FOR_CLIMBING),
        subsystems.coralIntakeArm);
  }
}
