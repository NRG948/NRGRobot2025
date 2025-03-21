/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static edu.wpi.first.math.geometry.Rotation2d.k180deg;
import static edu.wpi.first.math.geometry.Rotation2d.kZero;
import static frc.robot.parameters.Colors.PINK;
import static frc.robot.parameters.Colors.WHITE;
import static frc.robot.util.FieldUtils.getRobotPoseForNearestReefAprilTag;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;
import frc.robot.util.ReefPosition;
import java.util.Set;
import java.util.function.BiFunction;

/** A namespace for driver command factory methods. */
public final class DriveCommands {

  /** An enum used to select among various align to reef strategies. */
  public static enum AlignToReefSelector {
    /** Aligns to the reef position using the {@link AlignToReef} command. */
    USING_ALIGN_TO_REEF((s, p) -> new AlignToReef(s, p)),

    /* Aligns to the reef position using Pathplanner. */
    USING_ALIGN_TO_REEF_PP((s, p) -> alignToReefPP(s, p)),

    /** Aligns to the reef position using the {@link DriveToPose} command. */
    USING_DRIVE_STRAIGHT_TO_REEF(
        (s, p) -> {
          var pose = getRobotPoseForNearestReefAprilTag(s.drivetrain.getPosition(), p);

          return new DriveToPose(s.drivetrain, pose, Swerve.getMaxSpeed() * 0.3);
        }),

    USING_LATERAL_ALIGNMENT(
        (s, p) -> {
          return Commands.sequence(
              VisionCommands.waitForAprilTag(s.aprilTag, s.drivetrain),
              new ReefLateralAlignment(s, p),
              new DriveUntilHitObstacle(s, 1));
        });

    private final BiFunction<Subsystems, ReefPosition, Command> commandProducer;

    /**
     * Creates a new {@link AlignToReefSelector} enum variant.
     *
     * @param commandProducer A function that produces a command based on the subsystems and reef
     *     position.
     */
    private AlignToReefSelector(BiFunction<Subsystems, ReefPosition, Command> commandProducer) {
      this.commandProducer = commandProducer;
    }

    /**
     * Creates a new command based on the selected align to reef strategy.
     *
     * @param subsystems The subsystems container.
     * @param reefPosition The specified reef position.
     * @return A command that aligns to the specified reef position using the selected strategy.
     */
    public Command newCommand(Subsystems subsystems, ReefPosition reefPosition) {
      return commandProducer.apply(subsystems, reefPosition);
    }
  }

  @RobotPreferencesValue(column = 0, row = 1)
  public static final RobotPreferences.EnumValue<AlignToReefSelector> ALIGN_TO_REEF_STRATEGY =
      new RobotPreferences.EnumValue<>(
          "Drive", "AlignToReefStrategy", AlignToReefSelector.USING_ALIGN_TO_REEF);

  /**
   * Returns a command that resets the orientation of the drivetrain.
   *
   * @param subsystems The subsystems container.
   * @return
   */
  public static Command resetOrientation(Subsystems subsystems) {
    Swerve drivetrain = subsystems.drivetrain;

    return Commands.runOnce(
            () -> drivetrain.resetOrientation(FieldUtils.isRedAlliance() ? k180deg : kZero),
            drivetrain)
        .withName("ResetOrientation");
  }

  /**
   * Returns a command that aligns the robot to the specified reef position.
   *
   * @param subsystems The subsystems container.
   * @param reefPosition The specified reef position.
   * @return A command that aligns the robot to the specified reef position.
   */
  public static Command alignToReefPosition(Subsystems subsystems, ReefPosition reefPosition) {
    StatusLED statusLEDs = subsystems.statusLEDs;

    return Commands.parallel(
            new BlinkColor(statusLEDs, PINK).asProxy(),
            Commands.sequence(
                Commands.defer(
                    () -> ALIGN_TO_REEF_STRATEGY.getValue().newCommand(subsystems, reefPosition),
                    Set.of(subsystems.drivetrain)), //
                new BlinkColor(statusLEDs, WHITE).asProxy()))
        .withName(String.format("AlignToReefPosition(%s)", reefPosition.name()));
  }

  /**
   * Returns a command that aligns the robot to the coral station while intaking.
   *
   * @param subsystems The subsystems container.
   * @return A command that aligns the robot to the coral station while intaking.
   */
  public static Command alignToCoralStation(Subsystems subsystems) {
    return Commands.parallel(
            new AlignToCoralStation(subsystems), //
            CoralCommands.intakeUntilCoralDetected(subsystems))
        .withName("AlignToCoralStationWithIntake");
  }

  /**
   * Returns a command that interrupts all subsystems.
   *
   * @param subsystems The subsystems container.
   * @return A command that interrupts all subsystems.
   */
  public static Command interruptAll(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.disableAll(), subsystems.getAll())
        .withName("InterruptAll");
  }

  /**
   * Returns a command to follow the path to the specified branch of the nearest reef side.
   *
   * @param subsystems The Subsystems container.
   * @param reefPosition The target reef branch (left or right).
   * @return A command to follow the path to the specified branch of the nearest reef side.
   */
  public static Command alignToReefPP(Subsystems subsystems, ReefPosition reefPosition) {
    Swerve drivetrain = subsystems.drivetrain;

    var targetPose = getRobotPoseForNearestReefAprilTag(drivetrain.getPosition(), reefPosition);
    System.out.println("TARGET pose: " + targetPose);
    System.out.println("Target Branch: " + reefPosition);

    SwerveDriveParameters currentSwerveParameters = Swerve.PARAMETERS.getValue();

    return AutoBuilder.pathfindToPose(
            targetPose,
            new PathConstraints(
                Swerve.getMaxSpeed() * 0.3,
                Swerve.getMaxAcceleration(),
                currentSwerveParameters.getMaxRotationalSpeed() * 0.3,
                currentSwerveParameters.getMaxRotationalAcceleration()))
        .withName("AlignToReefPP");
  }
}
