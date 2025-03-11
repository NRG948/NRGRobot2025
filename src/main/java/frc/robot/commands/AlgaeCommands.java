/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.AlgaeArmState.INTAKE;
import static frc.robot.parameters.AlgaeArmState.OUTTAKE;
import static frc.robot.parameters.AlgaeArmState.STOWED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.AlgaeArmState;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Subsystems;

/** A namespace for algae command factory methods. */
public final class AlgaeCommands {

  /** Returns a command to intake algae. */
  public static Command intakeAlgae(Subsystems subsystems) {
    return setAlgaeState(subsystems, INTAKE).withName("IntakeAlgae");
  }

  /** Returns a command to outtake algae. */
  public static Command outtakeAlgae(Subsystems subsystems) {
    return setAlgaeState(subsystems, OUTTAKE).withName("OuttakeAlgae");
  }

  /** Returns a command that stops and stows the intake. */
  public static Command stopAndStowAlgaeIntake(Subsystems subsystems) {
    return setAlgaeState(subsystems, STOWED).withName("StopAndStowAlgaeIntake");
  }

  /** Returns a command that sets the algae arm+intake to a given state and then idles both. */
  private static Command setAlgaeState(Subsystems subsystems, AlgaeArmState state) {
    if (subsystems.algaeArm.isEmpty() || subsystems.algaeGrabber.isEmpty()) {
      return Commands.none();
    }

    var grabber = subsystems.algaeGrabber.get();
    var arm = subsystems.algaeArm.get();

    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> arm.setGoalAngle(state.armAngle()), arm),
            Commands.runOnce(() -> grabber.setGoalVelocity(state.grabberVelocity()), grabber)),
        Commands.idle(arm, grabber));
  }

  /** Returns a command that removes algae at the given reef level. */
  public static Command removeAlgaeAtLevel(Subsystems subsystems, ElevatorLevel elevatorLevel) {
    Elevator elevator = subsystems.elevator;
    Arm coralArm = subsystems.coralArm;
    CoralRoller coralRoller = subsystems.coralRoller;

    return Commands.sequence(
            Commands.parallel(
                ElevatorCommands.goToElevatorLevel(subsystems, elevatorLevel),
                CoralCommands.setArmAngleForReefLevel(subsystems, elevatorLevel),
                CoralCommands.outtakeCoral(subsystems)),
            Commands.idle(elevator, coralArm, coralRoller))
        .finallyDo(coralRoller::disable)
        .withName(String.format("RemoveAlgaeAtLevel(%s)", elevatorLevel.name()));
  }
}
