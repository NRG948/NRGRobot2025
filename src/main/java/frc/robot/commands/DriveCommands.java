/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.ColorConstants;
import frc.robot.commands.AlignToReef.ReefBranch;
import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

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
   * Returns a command that aligns the robot to the specified branch of the reef.
   *
   * @param subsystems The subsystems container.
   * @param branch The specified branch.
   * @return A command that aligns the robot to the specified branch of the reef.
   */
  public static Command alignToBranch(Subsystems subsystems, ReefBranch branch) {
    return Commands.sequence(
            new AlignToReef(subsystems, branch),
            new BlinkColor(subsystems.statusLEDs, ColorConstants.WHITE, 1).repeatedly())
        .withName(String.format("AlignToReef(%s)", branch.name()));
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
   * @param targetReefBranch The target reef branch (left or right).
   * @return A command to follow the path to the specified branch of the nearest reef side.
   */
  public static Command alignToReefPP(Subsystems subsystems, ReefBranch targetReefBranch) {
    Swerve drivetrain = subsystems.drivetrain;
    Pose2d currentRobotPose = drivetrain.getPosition();
    int nearestTagId = findNearestReefTagID(currentRobotPose);
    Pose2d targetPose =
        Constants.VisionConstants.REEF_SCORING_POSES.get(
            new Pair<Integer, ReefBranch>(nearestTagId, targetReefBranch));
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

  // TODO: get tag based on vision
  private static int findNearestReefTagID(Pose2d robotPose) {
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    int startingId = alliance.equals(Alliance.Red) ? 6 : 17;
    int nearestId = -1;
    double minDist = Double.MAX_VALUE;
    for (int id = startingId; id < startingId + 6; id++) {
      Transform2d tagToRobot = layout.getTagPose(id).get().toPose2d().minus(robotPose);
      double dist = Math.hypot(tagToRobot.getX(), tagToRobot.getY());
      if (dist < minDist) {
        minDist = dist;
        nearestId = id;
      }
    }

    return nearestId;
  }
}
