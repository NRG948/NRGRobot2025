/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.ElevatorLevel.L4;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Subsystems;

/** A namespace for coral command factory methods. */
public final class CoralCommands {
  private static final double CORAL_DETECTION_DELAY = 0.08;

  /** Returns a command that intakes coral. */
  public static Command intakeCoral(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.coralRoller.intake(), subsystems.coralRoller)
        .withName("IntakeCoral");
  }

  /** Returns a command that outtakes coral. */
  public static Command outtakeCoral(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.coralRoller.outtake(), subsystems.coralRoller)
        .withName("OuttakeCoral");
  }

  /** Returns a command that intakes coral until it is detected. */
  public static Command intakeUntilCoralDetected(Subsystems subsystems) {
    return Commands.sequence(
            intakeCoral(subsystems),
            Commands.idle(subsystems.coralRoller).until(subsystems.coralRoller::hasCoral),
            Commands.waitSeconds(CORAL_DETECTION_DELAY),
            Commands.runOnce(subsystems.coralRoller::disable, subsystems.coralRoller))
        .handleInterrupt(subsystems.coralRoller::disable)
        .unless(subsystems.coralRoller::hasCoral)
        .withName("IntakeUntilCoralDetected");
  }

  /** Returns a command that outtakes coral until it is not detected. */
  public static Command outtakeUntilCoralNotDetected(Subsystems subsystems) {
    return Commands.sequence(
            outtakeCoral(subsystems),
            Commands.either(
                Commands.sequence(Commands.waitSeconds(0.5), stowArm(subsystems)),
                Commands.none(),
                () -> subsystems.elevator.isNearestToLevel(L4)),
            Commands.idle(subsystems.coralRoller).until(() -> !subsystems.coralRoller.hasCoral()),
            Commands.runOnce(subsystems.coralRoller::disable, subsystems.coralRoller))
        .withName("OuttakeUntilCoralNotDetected");
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
            () -> subsystems.coralArm.setGoalAngle(level.getArmAngle()), subsystems.coralArm)
        .withName(String.format("SetArmAngleForReefLevel(%s)", level.name()));
  }

  /** Returns a command that waits for coral arm to reach goal angle. */
  public static Command waitForArmToReachGoalAngle(Subsystems subsystems) {
    return Commands.idle(subsystems.coralArm)
        .until(subsystems.coralArm::atGoalAngle)
        .withName("WaitForArmToReachGoalAngle");
  }

  /** Returns a command to stow the coral arm. */
  public static Command stowArm(Subsystems subsystems) {
    return Commands.sequence(
            Commands.runOnce(
                () -> subsystems.coralArm.setGoalAngle(ElevatorLevel.STOWED.getArmAngle()),
                subsystems.coralArm),
            waitForArmToReachGoalAngle(subsystems),
            Commands.runOnce(() -> subsystems.coralArm.disable(), subsystems.coralArm))
        .withName("StowArm");
  }

  public static Command waitForElevatorToReachArmHeight(Subsystems subsystems) {
    return Commands.idle(subsystems.coralArm)
        .until(subsystems.elevator::isAboveSafeArmPivotHeight)
        .withName("waitForElevatorToReachArmHeight");
  }
}
