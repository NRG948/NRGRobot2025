/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.nrg948.autonomous.AutonomousCommandMethod;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import java.io.File;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import org.javatuples.LabelValue;

/** A namespace for autonomous command factory methods. */
public final class Autos {
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /** Returns an autonomous command that does nothing. */
  @AutonomousCommandMethod(name = "None", isDefault = true)
  public static Command none(Subsystems subsystem) {
    return Commands.none();
  }

  /**
   * Returns a collection of label-value pairs mapping autonomous routine names to autonomous
   * commands define using PPPathplannner.
   */
  @AutonomousCommandGenerator
  public static Collection<LabelValue<String, Command>> generatePathPlannerAutos(
      Subsystems subsystems) {
    File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    return Arrays.stream(autosDir.listFiles((file, name) -> name.endsWith(".auto")))
        .map((file) -> file.getName().split("\\.")[0])
        .sorted()
        .map(name -> new LabelValue<>(name, getPathPlannerAuto(subsystems, name)))
        .toList();
  }

  /**
   * Returns the PathPlanner auto command.
   *
   * @param subsystems Subsystems container.
   * @param name Name of the PathPlanner auto.
   * @return The PathPlanner auto command.
   */
  public static Command getPathPlannerAuto(Subsystems subsystems, String name) {
    Swerve driveTrain = subsystems.drivetrain;

    NamedCommands.registerCommands(getPathplannerEventMap(subsystems, name));

    return Commands.defer(() -> newPathPlannerAuto(name), Set.of(driveTrain)).withName(name);
  }

  /**
   * Returns a {@link PathPlannnerAuto} instance for the given Pathplanner autonomous routine name.
   */
  private static Command newPathPlannerAuto(String name) {
    PathPlannerAuto pathPlannerAuto = new PathPlannerAuto(name);
    return pathPlannerAuto;
  }

  /**
   * Returns a map of event names to commands for the given Pathplanner autonomous routine name.
   *
   * @param subsystems Subsystems container.
   * @param pathGroupName Name of the pathplanner autonomous routine.
   * @return A map of event names to commands.
   */
  private static Map<String, Command> getPathplannerEventMap(
      Subsystems subsystems, String pathGroupName) {

    Map<String, Command> eventMaps =
        new HashMap<String, Command>(); // TODO: Replace placeholder parameters

    // TODO: Replace placeholder commands and parameters
    eventMaps.put(
        "Remove Algae L2", AlgaeCommands.removeAlgaeAtLevel(subsystems, ElevatorLevel.AlgaeL2));
    eventMaps.put(
        "Remove Algae L3", AlgaeCommands.removeAlgaeAtLevel(subsystems, ElevatorLevel.AlgaeL3));

    eventMaps.put("Coral Intake", CoralCommands.intakeUntilCoralDetected(subsystems));
    eventMaps.put("Coral Outtake", CoralCommands.outtakeUntilCoralNotDetected(subsystems));

    eventMaps.put("Elevator L1", ElevatorCommands.goToElevatorLevel(subsystems, ElevatorLevel.L1));
    eventMaps.put("Elevator L2", ElevatorCommands.goToElevatorLevel(subsystems, ElevatorLevel.L2));
    eventMaps.put("Elevator L3", ElevatorCommands.goToElevatorLevel(subsystems, ElevatorLevel.L3));
    eventMaps.put("Elevator L4", ElevatorCommands.goToElevatorLevel(subsystems, ElevatorLevel.L4));
    eventMaps.put("Elevator Dock", ElevatorCommands.stowElevator(subsystems));

    return eventMaps;
  }
}
