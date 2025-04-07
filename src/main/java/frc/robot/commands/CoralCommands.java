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
  private static final double CORAL_GRABBER_OUTTAKE_VELOCITY = -1.0;
  public static final double GROUND_INTAKE_INTAKE_ANGLE =
      CORAL_GROUND_INTAKE_ARM_PARAMETERS.getMinAngleRad();
  public static final double GROUND_INTAKE_STOWED_ANGLE =
      CORAL_GROUND_INTAKE_ARM_PARAMETERS.getMaxAngleRad();
  public static final double GROUND_INTAKE_L1_ANGLE = Math.toRadians(45);

  public static final double CORAL_ROLLER_DETECTION_DELAY = CORAL_ARM_PARAMETERS.getRollerDelay();
  public static final double CORAL_GRABBER_DETECTION_DELAY =
      CORAL_GROUND_INTAKE_ARM_PARAMETERS.getRollerDelay();

  /** The delay for reversing the coral during auto centering. */
  private static final double AUTO_CENTER_BACKWARDS_SECONDS = 0.2;

  /** The delay for intaking the coral during auto centering. */
  private static final double AUTO_CENTER_FORWARDS_SECONDS = 1.0;

  /** The velocity for transfering the coral from the ground intake into the funnel. */
  private static final double CORAL_GRABBER_TRANSFER_VELOCITY = -0.7;

  /** The velocity for intaking the coral using the ground intake. */
  private static final double CORAL_GRABBER_INTAKE_VELOCITY = 1.0;

  /** The velocity for scoring L1 using the ground intake. */
  private static final double CORAL_GRABBER_L1_VELOCITY = -0.3;

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
  public static Command scoreToReefL2ThruL4(Subsystems subsystems) {
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

  /** Outtakes L1 coral using the ground intake. */
  public static Command scoreToReefL1(Subsystems subsystems) {
    var coralIntakeGrabber = subsystems.coralIntakeGrabber;
    return Commands.sequence(
            Commands.runOnce(
                () -> coralIntakeGrabber.setGoalVelocity(CORAL_GRABBER_L1_VELOCITY),
                coralIntakeGrabber))
        .withName("ScoreToReefL1");
  }

  public static Command scoreToReef(Subsystems subsystems) {
    var coralRoller = subsystems.coralRoller;
    var coralIntakeGrabber = subsystems.coralIntakeGrabber;
    return Commands.either(
            scoreToReefL2ThruL4(subsystems), scoreToReefL1(subsystems), coralRoller::hasCoral)
        .unless(() -> !coralRoller.hasCoral() && !coralIntakeGrabber.hasCoral())
        .withName("ScoreToReef");
  }

  public static Command stowAfterScoring(Subsystems subsystems) {
    return Commands.parallel(
            ElevatorCommands.stowElevatorAndArmForCoral(subsystems), stowGroundIntake(subsystems))
        .withName("StowAfterScoring");
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

  /**
   * Returns a command to wait for the elevator to reach the height at which the arm should pivot.
   */
  public static Command waitForElevatorToReachArmHeight(Subsystems subsystems) {
    Arm coralArm = subsystems.coralArm;
    Elevator elevator = subsystems.elevator;

    return Commands.idle(coralArm)
        .until(elevator::isAboveSafeArmPivotHeight)
        .withName("waitForElevatorToReachArmHeight");
  }

  /** Returns a command to "center" the coral in the coral arm by outtaking and re-intaking. */
  public static Command autoCenterCoral(Subsystems subsystems) {
    CoralRoller coralRoller = subsystems.coralRoller;

    return Commands.sequence(
            Commands.runOnce(
                () -> coralRoller.setGoalVelocity(CoralRoller.AUTO_CENTER_VELOCITY.getValue()),
                coralRoller),
            Commands.idle(coralRoller).withTimeout(AUTO_CENTER_BACKWARDS_SECONDS),
            intakeUntilCoralDetected(subsystems).withTimeout(AUTO_CENTER_FORWARDS_SECONDS))
        .finallyDo(coralRoller::disable)
        .unless(coralRoller::hasCoral)
        .withName("AutoCenterCoral");
  }

  /** Returns a command to intake the ground intake arm. */
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
        .unless(coralIntakeGrabber::hasCoral)
        .withName("IntakeFromGround");
  }

  /** Returns a command to stow the ground intake arm. */
  public static Command stowGroundIntake(Subsystems subsystems) {
    var coralIntakeArm = subsystems.coralIntakeArm;
    var coralIntakeGrabber = subsystems.coralIntakeGrabber;
    return Commands.sequence(
            Commands.runOnce(
                () -> coralIntakeArm.setGoalAngle(GROUND_INTAKE_STOWED_ANGLE), coralIntakeArm),
            Commands.runOnce(coralIntakeGrabber::disable, coralIntakeGrabber),
            Commands.idle(coralIntakeArm).until(coralIntakeArm::atGoalAngle))
        .withName("StowGroundIntake");
  }

  /** Returns a command to transfer coral from the ground intake arm to the funnel. */
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
                    // || !coralIntakeArm.isStowed()
                    || !coralArm.isStowed())
        .withName("TransferFromGroundIntake");
  }

  /** Returns a command to set the angle for L1 */
  public static Command extendToReefL1(Subsystems subsystems) {
    var coralIntakeArm = subsystems.coralIntakeArm;
    return Commands.sequence(
            Commands.runOnce(
                () -> coralIntakeArm.setGoalAngle(GROUND_INTAKE_L1_ANGLE), coralIntakeArm),
            Commands.idle(coralIntakeArm)
                .until(() -> coralIntakeArm.atGoalAngle(GROUND_INTAKE_L1_ANGLE)))
        .unless(() -> coralIntakeArm.atGoalAngle(GROUND_INTAKE_L1_ANGLE))
        .withName("ExtendToReefL1");
  }

  /** This command returns to manually outtake coral using the ground intake */
  public static Command manualGroundOuttake(Subsystems subsystems) {
    var coralIntakeGrabber = subsystems.coralIntakeGrabber;
    return Commands.sequence(
            Commands.runOnce(
                () -> coralIntakeGrabber.setGoalVelocity(CORAL_GRABBER_L1_VELOCITY),
                coralIntakeGrabber),
            Commands.idle(coralIntakeGrabber))
        .finallyDo(coralIntakeGrabber::disable)
        .withName("ManualGroundOuttake");
  }

  public static Command disableManualGroundOuttake(Subsystems subsystems) {
    var coralIntakeGrabber = subsystems.coralIntakeGrabber;
    return Commands.runOnce(() -> coralIntakeGrabber.disable(), coralIntakeGrabber);
  }

  public static Command stowAll(Subsystems subsystems) {
    return Commands.parallel(
        ElevatorCommands.stowElevatorAndArm(subsystems), stowGroundIntake(subsystems));
  }
}
