/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import static frc.robot.Constants.RobotConstants.CORAL_ARM_CENTER_Y_OFFSET;
import static frc.robot.Constants.RobotConstants.ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X;
import static frc.robot.Constants.VisionConstants.BRANCH_TO_REEF_APRILTAG;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * An enum that represents the reef alignment positions as viewed face on.
 *
 * @param yOffset The y offset from the center of the AprilTag to the reef position in the
 *     AprilTag's frame of reference.
 */
public enum ReefPosition {
  LEFT_BRANCH(-BRANCH_TO_REEF_APRILTAG),
  CENTER_REEF(0.0),
  RIGHT_BRANCH(BRANCH_TO_REEF_APRILTAG);

  private final double yOffset;
  private final Transform2d tagToRobot;

  ReefPosition(double yOffset) {
    this.yOffset = yOffset;
    this.tagToRobot =
        new Transform2d(
            ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X,
            CORAL_ARM_CENTER_Y_OFFSET + yOffset,
            Rotation2d.k180deg);
  }

  /**
   * Returns the y offset from the center of the AprilTag to the reef position in the AprilTag's
   * frame of reference.
   *
   * @return The y offset.
   */
  public double yOffset() {
    return yOffset;
  }

  /**
   * Returns the transform from the AprilTag to the robot's center of rotation.
   *
   * @return The transform from the AprilTag to the robot's center of rotation.
   */
  public Transform2d tagToRobot() {
    return tagToRobot;
  }
}
