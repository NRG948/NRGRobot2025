/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.commands.AlgaeCommands.removeAlgaeAtLevel;
import static frc.robot.commands.CoralAndElevatorCommands.raiseElevatorAndTipCoralArm;
import static frc.robot.commands.CoralCommands.intakeUntilCoralDetected;
import static frc.robot.commands.CoralCommands.outtakeUntilCoralNotDetected;
import static frc.robot.commands.ElevatorCommands.stowElevator;

import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.nrg948.autonomous.AutonomousCommandMethod;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.subsystems.Subsystems;
import java.io.File;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import org.javatuples.LabelValue;

/** A namespace for autonomous command factory methods. */
public final class Autos {
  private static final HashMap<String, Command> autosMap = new HashMap<String, Command>();

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /** Returns an autonomous command that does nothing. */
  @AutonomousCommandMethod(name = "None", isDefault = true)
  public static Command none(Subsystems subsystem) {
    return Commands.none().withName("None");
  }

  /**
   * Returns a collection of label-value pairs mapping autonomous routine names to autonomous
   * commands define using Pathplannner.
   */
  @AutonomousCommandGenerator
  public static Collection<LabelValue<String, Command>> generatePathPlannerAutos(
      Subsystems subsystems) {
    File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
    return Arrays.stream(autosDir.listFiles((file, name) -> name.endsWith(".auto")))
        .map((file) -> file.getName().split("\\.")[0])
        .sorted()
        .map(name -> new LabelValue<>(name, generatePathPlannerAuto(subsystems, name)))
        .toList();
  }

  /**
   * Returns the PathPlanner auto command.
   *
   * @param subsystems Subsystems container.
   * @param name Name of the PathPlanner auto.
   * @return The PathPlanner auto command.
   */
  public static Command generatePathPlannerAuto(Subsystems subsystems, String name) {
    NamedCommands.registerCommands(getPathplannerEventMap(subsystems, name));

    Set<Subsystem> requirements = new HashSet<>(Arrays.asList(subsystems.getManipulators()));
    requirements.add(subsystems.drivetrain);
    return Commands.defer(() -> getPathPlannerAuto(name), requirements).withName(name);
  }

  /**
   * Returns the PathPlanner auto command from the autosMap, creating one if it hasn't already been
   * preloaded.
   *
   * @param name Name of the PathPlanner auto.
   * @return The PathPlanner auto command.
   */
  private static Command getPathPlannerAuto(String name) {
    Command autoCommand = autosMap.get(name);
    if (autoCommand == null) {
      autoCommand = newPathPlannerAuto(name);
    }
    return autoCommand;
  }

  /**
   * Preloads the specified PathPlanner command.
   *
   * @param auto The auto to preload.
   */
  public static void preloadAuto(Command auto) {
    String autoName = auto.getName();
    Command autoCommand = newPathPlannerAuto(autoName);
    autosMap.put(autoName, autoCommand);
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

    Map<String, Command> eventMaps = new HashMap<String, Command>();

    eventMaps.put("Remove Algae L2", removeAlgaeAtLevel(subsystems, ElevatorLevel.AlgaeL2));
    eventMaps.put("Remove Algae L3", removeAlgaeAtLevel(subsystems, ElevatorLevel.AlgaeL3));

    eventMaps.put("Coral Intake", intakeUntilCoralDetected(subsystems));
    eventMaps.put("Coral Outtake", outtakeUntilCoralNotDetected(subsystems));

    eventMaps.put("Elevator L1", raiseElevatorAndTipCoralArm(subsystems, ElevatorLevel.L1));
    eventMaps.put("Elevator L2", raiseElevatorAndTipCoralArm(subsystems, ElevatorLevel.L2));
    eventMaps.put("Elevator L3", raiseElevatorAndTipCoralArm(subsystems, ElevatorLevel.L3));
    eventMaps.put("Elevator L4", raiseElevatorAndTipCoralArm(subsystems, ElevatorLevel.L4));
    eventMaps.put("Elevator Dock", stowElevator(subsystems));

    return eventMaps;
  }
}
