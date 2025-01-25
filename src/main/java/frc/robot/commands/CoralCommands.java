/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Subsystems;

/** Add your docs here. */
public class CoralCommands {
  /** Returns a command that intakes coral. */
  public static Command intakeCoral(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.coralRoller.intake(), subsystems.coralRoller);
  }

  /** Returns a command that outtakes coral. */
  public static Command outtakeCoral(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.coralRoller.outtake(), subsystems.coralRoller);
  }

  /**
   * Returns a command that sets the arm angle of the coral arm.
   *
   * @param subsystems The subsystems container.
   * @param level The reef level.
   * @return
   */
  public static Command setArmAngleForReefLevel(Subsystems subsystems, ElevatorLevel level) {
    return Commands.runOnce(
        () -> subsystems.coralArm.setGoalAngle(level.getPivotAngle()), subsystems.coralArm);
  }

  /** Returns a command that waits for coral arm to reach goal angle. */
  public static Command waitForArmToReachGoalAngle(Subsystems subsystems) {
    return Commands.idle(subsystems.coralArm).until(subsystems.coralArm::atGoalAngle);
  }

  /** Returns a command to stow the coral arm. */
  public static Command stowArm(Subsystems subsystems) {
    return Commands.sequence(
        Commands.runOnce(
            () -> subsystems.coralArm.setGoalAngle(ElevatorLevel.STOWED.getPivotAngle()),
            subsystems.coralArm),
        waitForArmToReachGoalAngle(subsystems),
        Commands.runOnce(() -> subsystems.coralArm.disable(), subsystems.coralArm));
  }

  /**
   * Returns the command sequence that outtakes the coral until not detected and then stows the
   * elevator and arm.
   *
   * @param subsystems
   * @return
   */
  public static Command deliverCoral(Subsystems subsystems) {
    return Commands.none();
  }
}
