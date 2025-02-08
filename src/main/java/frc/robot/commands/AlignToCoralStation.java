/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.ODOMETRY_CENTER_TO_REAR_BUMPER_DELTA_X;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.FieldUtils;

/** Command that aligns the robot to the nearest coral station. */
public class AlignToCoralStation extends AlignToPose {
  /** Creates a new AlignToCoralStation. */
  public AlignToCoralStation(Subsystems subsystems) {
    super(subsystems);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Figure out the robot's target pose.
    Pose2d currentRobotPose = drivetrain.getPosition();
    Pose2d nearestTagPose = currentRobotPose.nearest(FieldUtils.getCoralStationAprilTags());
    double xOffset = ODOMETRY_CENTER_TO_REAR_BUMPER_DELTA_X;
    double yOffset = 0;

    targetPose = nearestTagPose.plus(new Transform2d(xOffset, yOffset, Rotation2d.k180deg));

    // Set up the PID controllers to drive to the target pose.
    super.initialize();
  }
}
