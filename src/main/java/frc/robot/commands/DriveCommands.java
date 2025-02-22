/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.Colors.WHITE;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.commands.AlignToReef.ReefPosition;
import frc.robot.parameters.Colors;
import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

/** A namespace for driver command factory methods. */
public final class DriveCommands {
  /**
   * Returns a command that resets the orientation of the drivetrain.
   *
   * @param subsystems The subsystems container.
   * @return
   */
  public static Command resetOrientation(Subsystems subsystems) {
    return Commands.runOnce(() -> subsystems.drivetrain.resetOrientation(new Rotation2d()))
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
    return Commands.sequence(
            new AlignToReef(subsystems, reefPosition), new BlinkColor(subsystems.statusLEDs, WHITE))
        .withName(String.format("AlignToReef(%s)", reefPosition.name()));
  }

  /**
   * Returns a command that alligns the robot to the Center of the Coral station.
   *
   * @param subsystems The subsystems container.
   * @return A command that alligns the robot to the Center of the Coral station.
   */
  public static Command alignToCoralStationCenter(Subsystems subsystems) {
    return Commands.sequence(
            new AlignToCoralStation(subsystems),
            new BlinkColor(subsystems.statusLEDs, Colors.GREEN))
        .withName("alignToCoralStationCenter");
  }

  /**
   * Returns a command that interrupts all subsystems.
   *
   * @param subsystems The subsystems container.
   * @return A command that interrupts all subsystems.
   */
  public static Command interruptAll(Subsystems subsystems) {
    return Commands.runOnce(() -> {}, subsystems.getAll()).withName("InterruptAll");
  }

  /**
   * Returns a command to follow the path to the specified branch of the nearest reef side.
   *
   * @param subsystems The Subsystems container.
   * @param targetReefPosition The target reef branch (left or right).
   * @return A command to follow the path to the specified branch of the nearest reef side.
   */
  public static Command alignToReefPP(Subsystems subsystems, ReefPosition targetReefPosition) {
    Swerve drivetrain = subsystems.drivetrain;

    Pose2d currentRobotPose = drivetrain.getPosition();
    Pose2d nearestTagPose = currentRobotPose.nearest(FieldUtils.getReefAprilTags());
    double v, h, d;
    v = Constants.RobotConstants.ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X;
    h = Constants.RobotConstants.CORAL_OFFSET_Y;
    d = Constants.VisionConstants.BRANCH_TO_REEF_APRILTAG;

    var targetPose =
        nearestTagPose.plus(
            new Transform2d(
                v,
                (targetReefPosition.equals(ReefPosition.RIGHT_BRANCH) ? d : -d) - h,
                Rotation2d.k180deg));
    System.out.println("TARGET pose: " + targetPose);
    System.out.println("Target Branch: " + targetReefPosition);

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
