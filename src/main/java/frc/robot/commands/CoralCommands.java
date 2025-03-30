/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.Colors.YELLOW;
import static frc.robot.subsystems.Arm.CORAL_ARM_PARAMETERS;
import static frc.robot.subsystems.Arm.CORAL_GROUND_INTAKE_ARM_PARAMETERS;
import static frc.robot.subsystems.CoralRoller.INTAKE_VELOCITY;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;
import java.util.Set;

/** A namespace for coral command factory methods. */
public final class CoralCommands {
  public static final double GROUND_INTAKE_INTAKE_ANGLE =
      CORAL_GROUND_INTAKE_ARM_PARAMETERS.getMinAngleRad();
  public static final double GROUND_INTAKE_STOWED_ANGLE =
      CORAL_GROUND_INTAKE_ARM_PARAMETERS.getMaxAngleRad();

  public static final double CORAL_ROLLER_DETECTION_DELAY = CORAL_ARM_PARAMETERS.getRollerDelay();
  public static final double CORAL_GRABBER_DETECTION_DELAY =
      CORAL_GROUND_INTAKE_ARM_PARAMETERS.getRollerDelay();

  /** The delay for reversing the coral during auto centering. */
  private static final double AUTO_CENTER_BACKWARDS_SECONDS = 0.2;

  /** The delay for intaking the coral during auto centering. */
  private static final double AUTO_CENTER_FORWARDS_SECONDS = 1.0;

  /** The velocity for transfering the coral from the ground intake into the funnel */
  private static final double CORAL_GRABBER_TRANSFER_VELOCITY = -1.0;

  /** The velocity for intaking the coral using the ground intake */
  private static final double CORAL_GRABBER_INTAKE_VELOCITY = 1.0;

  /** Returns a command that intakes coral. */
  public static Command intakeCoral(Subsystems subsystems) {
    CoralRoller coralRoller = subsystems.coralRoller;

    return Commands.runOnce(
            () -> coralRoller.setGoalVelocity(INTAKE_VELOCITY.getValue()), coralRoller)
        .withName("IntakeCoral");
  }

  /** Returns a command that outtakes coral. */
  public static Command outtakeCoral(Subsystems subsystems) {
    CoralRoller coralRoller = subsystems.coralRoller;

    return Commands.runOnce(
            () ->
                coralRoller.setGoalVelocity(
                    subsystems.elevator.getCurrentElevatorLevel().getOuttakeSpeed()),
            coralRoller)
        .withName("OuttakeCoral");
  }

  /** Returns a command that intakes coral until it is detected. */
  public static Command intakeUntilCoralDetected(Subsystems subsystems) {
    CoralRoller coralRoller = subsystems.coralRoller;
    StatusLED statusLEDs = subsystems.statusLEDs;

    return Commands.parallel(
            new BlinkColor(statusLEDs, YELLOW).asProxy(),
            Commands.sequence(
                intakeCoral(subsystems),
                Commands.idle(coralRoller).until(coralRoller::hasCoral),
                Commands.waitSeconds(CORAL_ROLLER_DETECTION_DELAY),
                Commands.runOnce(coralRoller::disable, coralRoller)))
        .finallyDo(coralRoller::disable)
        .unless(coralRoller::hasCoral)
        .withName("IntakeUntilCoralDetected");
  }

  /** Returns a command that outtakes coral until it is not detected. */
  public static Command outtakeUntilCoralNotDetected(Subsystems subsystems) {
    CoralRoller coralRoller = subsystems.coralRoller;
    Elevator elevator = subsystems.elevator;

    return Commands.sequence(
            outtakeCoral(subsystems),
            Commands.defer(
                () -> Commands.waitSeconds(elevator.getCurrentElevatorLevel().getOuttakeDelay()),
                Set.of()),
            stowArm(subsystems),
            Commands.idle(coralRoller).until(() -> !coralRoller.hasCoral()))
        .finallyDo(coralRoller::disable)
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
    Arm coralArm = subsystems.coralArm;

    return Commands.runOnce(() -> coralArm.setGoalAngle(level.getArmAngle()), coralArm)
        .withName(String.format("SetArmAngleForReefLevel(%s)", level.name()));
  }

  /** Returns a command that waits for coral arm to reach goal angle. */
  public static Command waitForArmToReachGoalAngle(Subsystems subsystems) {
    Arm coralArm = subsystems.coralArm;

    return Commands.idle(coralArm)
        .until(coralArm::atGoalAngle)
        .withName("WaitForArmToReachGoalAngle");
  }

  /** Returns a command to stow the coral arm. */
  public static Command stowArm(Subsystems subsystems) {
    Arm coralArm = subsystems.coralArm;

    return Commands.sequence(
            Commands.runOnce(
                () -> coralArm.setGoalAngle(ElevatorLevel.STOWED.getArmAngle()), coralArm),
            waitForArmToReachGoalAngle(subsystems))
        .finallyDo(coralArm::disable)
        .withName("StowArm");
  }

  public static Command waitForElevatorToReachArmHeight(Subsystems subsystems) {
    Arm coralArm = subsystems.coralArm;
    Elevator elevator = subsystems.elevator;

    return Commands.idle(coralArm)
        .until(elevator::isAboveSafeArmPivotHeight)
        .withName("waitForElevatorToReachArmHeight");
  }

  public static Command autoCenterCoral(Subsystems subsystems) {
    CoralRoller coralRoller = subsystems.coralRoller;

    return Commands.sequence(
            Commands.runOnce(
                () -> coralRoller.setGoalVelocity(CoralRoller.AUTO_CENTER_VELOCITY.getValue()),
                coralRoller),
            Commands.idle(coralRoller).withTimeout(AUTO_CENTER_BACKWARDS_SECONDS),
            intakeUntilCoralDetected(subsystems).withTimeout(AUTO_CENTER_FORWARDS_SECONDS))
        .finallyDo(coralRoller::disable)
        .unless(coralRoller::hasCoral);
  }

  public static Command intakeFromGround(Subsystems subsystems) {
    var coralIntakeArm = subsystems.coralIntakeArm;
    var coralIntakeGrabber = subsystems.coralIntakeGrabber;
    return Commands.sequence(
            Commands.runOnce(
                () -> coralIntakeArm.setGoalAngle(GROUND_INTAKE_INTAKE_ANGLE), coralIntakeArm),
            Commands.runOnce(
                () -> coralIntakeGrabber.setGoalVelocity(CORAL_GRABBER_INTAKE_VELOCITY),
                coralIntakeGrabber),
            Commands.idle(coralIntakeArm, coralIntakeGrabber).until(coralIntakeGrabber::hasCoral),
            Commands.waitSeconds(CORAL_GRABBER_DETECTION_DELAY))
        .finallyDo(coralIntakeGrabber::disable)
        .unless(coralIntakeGrabber::hasCoral);
  }

  public static Command stowGroundIntake(Subsystems subsystems) {
    var coralIntakeArm = subsystems.coralIntakeArm;
    var coralIntakeGrabber = subsystems.coralIntakeGrabber;
    return Commands.parallel(
        Commands.runOnce(
            () -> coralIntakeArm.setGoalAngle(GROUND_INTAKE_STOWED_ANGLE), coralIntakeArm),
        Commands.runOnce(coralIntakeGrabber::disable, coralIntakeGrabber));
  }

  public static Command transferFromGroundIntake(Subsystems subsystems) {
    var coralIntakeArm = subsystems.coralIntakeArm;
    var coralIntakeGrabber = subsystems.coralIntakeGrabber;
    var coralArm = subsystems.coralArm;
    return Commands.parallel(
            intakeUntilCoralDetected(subsystems),
            Commands.sequence(
                Commands.runOnce(
                    () -> coralIntakeGrabber.setGoalVelocity(CORAL_GRABBER_TRANSFER_VELOCITY),
                    coralIntakeGrabber),
                Commands.idle(coralIntakeGrabber, coralIntakeArm)
                    .until(() -> !coralIntakeGrabber.hasCoral()),
                Commands.waitSeconds(CORAL_GRABBER_DETECTION_DELAY)))
        .finallyDo(coralIntakeGrabber::disable)
        .unless(
            () ->
                !coralIntakeGrabber.hasCoral()
                    || !coralIntakeArm.isStowed()
                    || !coralArm.isStowed());
  }
}
