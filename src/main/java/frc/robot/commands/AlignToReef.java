/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.CORAL_OFFSET_Y;
import static frc.robot.Constants.RobotConstants.ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.FieldUtils;
import frc.robot.util.ReefPosition;

/**
 * A {@link Command} that autonomous drives and aligns the robot to the specified position of the
 * nearest reef side.
 */
public class AlignToReef extends AlignToPose {

  private final ReefPosition reefPosition;

  /** Creates a new {@link AlignToReef} command. */
  public AlignToReef(Subsystems subsystems, ReefPosition reefPosition) {
    super(subsystems);
    setName(String.format("AlignToReef(%s)", reefPosition.name()));
    this.reefPosition = reefPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Figure out the robot's desired target pose relative to the nearest reef April Tag.
    Pose2d currentRobotPose = drivetrain.getPosition();
    Pose2d nearestTagPose = currentRobotPose.nearest(FieldUtils.getReefAprilTags());
    double xOffset = ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X;
    double yOffset = reefPosition.yOffset() - CORAL_OFFSET_Y;
    targetPose = nearestTagPose.plus(new Transform2d(xOffset, yOffset, Rotation2d.k180deg));

    super.initialize();
  }
}
