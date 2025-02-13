/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.CORAL_OFFSET_Y;
import static frc.robot.Constants.RobotConstants.ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X;
import static frc.robot.Constants.VisionConstants.BRANCH_TO_REEF_APRILTAG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.FieldUtils;

/**
 * A {@link Command} that autonomous drives and aligns the robot to the specified position of the
 * nearest reef side.
 */
public class AlignToReef extends AlignToPose {

  /** An enum that represents the reef alignment positions as viewed face on. */
  public enum ReefPosition {
    LEFT_BRANCH,
    CENTER,
    RIGHT_BRANCH
  }

  private final ReefPosition targetReefPosition;

  /** Creates a new {@link AlignToReef} command. */
  public AlignToReef(Subsystems subsystems, ReefPosition targetReefPosition) {
    super(subsystems);
    setName(String.format("AlignToReef(%s)", targetReefPosition.name()));
    this.targetReefPosition = targetReefPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Figure out the robot's target pose.
    Pose2d currentRobotPose = drivetrain.getPosition();
    Pose2d nearestTagPose = currentRobotPose.nearest(FieldUtils.getReefAprilTags());
    double xOffset = ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X;
    double yOffset = -CORAL_OFFSET_Y;
    switch (targetReefPosition) {
      case RIGHT_BRANCH:
        yOffset += BRANCH_TO_REEF_APRILTAG;
        break;
      case LEFT_BRANCH:
        yOffset -= BRANCH_TO_REEF_APRILTAG;
        break;
      case CENTER:
        break;
    }

    targetPose = nearestTagPose.plus(new Transform2d(xOffset, yOffset, Rotation2d.k180deg));
    System.out.println("Target Position: " + targetReefPosition);

    super.initialize();
  }
}
