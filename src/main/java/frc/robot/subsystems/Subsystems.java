/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.parameters.AlgaeArmState;
import frc.robot.parameters.ElevatorLevel;
import frc.robot.util.MotorIdleMode;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/** Contains all robot subsystems. */
public class Subsystems {
  public final Swerve drivetrain = new Swerve();
  public final Elevator elevator = new Elevator();

  public final Arm coralArm = new Arm(Arm.CORAL_ARM.getValue());
  public final CoralRoller coralRoller = new CoralRoller();

  public final Optional<Arm> algaeArm;
  public final Optional<AlgaeGrabber> algaeGrabber;

  public final Climber climber = new Climber();

  public final StatusLED statusLEDs = new StatusLED();

  public final Optional<AprilTag> frontCamera;
  public final Optional<AprilTag> backCamera;

  public final QuestNav questNav = new QuestNav();

  private final Subsystem[] all;
  private final Subsystem[] manipulators;

  private Map<String, StringLogEntry> commandLogger;

  /** Constructs the robot subsystems container. */
  public Subsystems() {
    // Add all manipulator subsystems to the `manipulators` list.
    var manipulators = new ArrayList<Subsystem>(Arrays.asList(elevator, coralArm, coralRoller));

    // Add all non-manipulator subsystems to the `all` list.
    var all = new ArrayList<Subsystem>(Arrays.asList(drivetrain, statusLEDs, climber, questNav));

    var visionParams = RobotContainer.PARAMETERS.getValue().visionParameters();

    // Add optional subsystems to the appropriate list.
    frontCamera =
        visionParams
            .robotToFrontCamera()
            .flatMap(
                (t) -> newOptionalSubsystem(AprilTag.class, AprilTag.ENABLED, "FrontCamera", t));

    frontCamera.ifPresent((s) -> all.add(s));

    backCamera =
        visionParams
            .robotToBackCamera()
            .flatMap(
                (t) -> newOptionalSubsystem(AprilTag.class, AprilTag.ENABLED, "BackCamera", t));

    backCamera.ifPresent((s) -> all.add(s));

    algaeArm = newOptionalSubsystem(Arm.class, Arm.ENABLE_ALGAE_ARM, Arm.ALGAE_ARM.getValue());
    algaeArm.ifPresent((s) -> manipulators.add(s));

    algaeGrabber = newOptionalSubsystem(AlgaeGrabber.class, Arm.ENABLE_ALGAE_ARM);
    algaeGrabber.ifPresent((s) -> manipulators.add(s));

    // Add all manipulator subsystems to the `all` list.
    all.addAll(manipulators);

    // Convert the lists to arrays.
    this.all = all.toArray(Subsystem[]::new);
    this.manipulators = manipulators.toArray(Subsystem[]::new);

    // Logs the active command on each subsystem.
    commandLogger =
        Arrays.stream(this.all)
            .collect(
                Collectors.toMap(
                    Subsystem::getName,
                    s ->
                        new StringLogEntry(
                            DataLogManager.getLog(),
                            String.format("/%s/ActiveCommand", s.getName()))));
    CommandScheduler scheduler = CommandScheduler.getInstance();
    scheduler.onCommandInitialize(
        (cmd) -> {
          cmd.getRequirements().stream()
              .forEach((s) -> commandLogger.get(s.getName()).append(cmd.getName()));
        });
    scheduler.onCommandFinish(
        (cmd) -> {
          cmd.getRequirements().stream().forEach((s) -> commandLogger.get(s.getName()).append(""));
        });
  }

  /** Returns an array of all subsystems. */
  public Subsystem[] getAll() {
    return all;
  }

  /** Returns an array of all manipulator subsystems. */
  public Subsystem[] getManipulators() {
    return manipulators;
  }

  /**
   * Creates a new optional subsystem.
   *
   * @param <T> The of subsystem.
   * @param subsystemClass The subsystem class.
   * @param enabled The preferences value indicating whether the subsystem is enabled.
   * @param initArgs The arguments to pass to the subsystem's constructor.
   * @return Returns a non-empty {@link Optional} instance if the subsystem is enabled. Otherwise,
   *     this method returns {@link Optional#empty}.
   */
  private static <T extends Subsystem> Optional<T> newOptionalSubsystem(
      Class<T> subsystemClass, RobotPreferences.BooleanValue enabled, Object... initArgs) {
    if (!enabled.getValue()) {
      return Optional.empty();
    }

    Class<?>[] initArgClasses = Stream.of(initArgs).map(Object::getClass).toArray(Class<?>[]::new);

    try {
      return Optional.of(subsystemClass.getConstructor(initArgClasses).newInstance(initArgs));
    } catch (InstantiationException
        | IllegalAccessException
        | IllegalArgumentException
        | InvocationTargetException
        | SecurityException e) {
      System.err.printf(
          "ERROR: An unexpected exception was caught while creating an instance of %s.%n",
          subsystemClass.getName());
      e.printStackTrace();
      return Optional.empty();
    } catch (NoSuchMethodException e) {
      System.err.printf(
          "ERROR: The class is missing constructor %s(%s).%n",
          subsystemClass.getName(),
          Stream.of(initArgClasses).map(Class::getSimpleName).collect(Collectors.joining(", ")));
      e.printStackTrace();
      return Optional.empty();
    }
  }

  public void setInitialStates() {
    coralArm.setGoalAngle(ElevatorLevel.STOWED.getArmAngle());
    algaeArm.ifPresent(algaeArm -> algaeArm.setGoalAngle(AlgaeArmState.STOWED.armAngle()));
  }

  /** Disables the specified subsystems implementing the {@link ActiveSubsystem} interface. */
  private void disableSubsystems(Subsystem[] subsystems) {
    for (Subsystem subsystem : subsystems) {
      if (subsystem instanceof ActiveSubsystem) {
        ActiveSubsystem.class.cast(subsystem).disable();
      }
    }
  }

  /** Disables all subsystems implementing the {@link ActiveSubsystem} interface. */
  public void disableAll() {
    disableSubsystems(all);
  }

  /** Disables all manipulator subsystems implementing the {@link ActiveSubsystem} interface. */
  public void disableManipulators() {
    disableSubsystems(manipulators);
  }

  /**
   * Sets the idle mode of the motors for all subsystems implementing the {@link ActiveSubsystem}
   * interface.
   */
  public void setIdleMode(MotorIdleMode idleMode) {
    for (Subsystem subsystem : all) {
      if (subsystem instanceof ActiveSubsystem) {
        ActiveSubsystem.class.cast(subsystem).setIdleMode(idleMode);
      }
    }
  }

  /**
   * Adds Shuffleboard tabs for all subsystems implementing the {@link ShuffleboardProducer}
   * interface.
   */
  public void initShuffleboard() {
    for (Subsystem subsystem : all) {
      if (subsystem instanceof ShuffleboardProducer) {
        ShuffleboardProducer.class.cast(subsystem).addShuffleboardTab();
      }
    }
  }

  /** Called to perform periodic actions. */
  public void periodic() {
    frontCamera.ifPresent(this::updateEstimatedPose);
    backCamera.ifPresent(this::updateEstimatedPose);
  }

  /**
   * Updates the estimated pose of the robot based on the given camera.
   *
   * @param camera The {@link AprilTag} subsystem managing the camera to use for updating the
   *     estimated pose.
   */
  private void updateEstimatedPose(AprilTag camera) {
    var visionEst = camera.getEstimateGlobalPose();

    visionEst.ifPresent(
        (est) -> {
          var estPose = est.estimatedPose.toPose2d();
          var estStdDevs = camera.getEstimationStdDevs();

          drivetrain.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
        });
  }
}
