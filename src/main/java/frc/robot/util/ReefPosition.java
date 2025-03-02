/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import static frc.robot.Constants.VisionConstants.BRANCH_TO_REEF_APRILTAG;

/** An enum that represents the reef alignment positions as viewed face on. */
public enum ReefPosition {
  LEFT_BRANCH(-BRANCH_TO_REEF_APRILTAG),
  CENTER_REEF(0),
  RIGHT_BRANCH(BRANCH_TO_REEF_APRILTAG);

  private double yOffset;

  /**
   * Creates a new {@link ReefPosition} enum.
   *
   * @param yOffset The y offset from the center of the AprilTag to the reef position in the
   *     AprilTag's frame of reference.
   */
  private ReefPosition(double yOffset) {
    this.yOffset = yOffset;
  }

  /**
   * Returns the y offset from the center of the AprilTag to the reef position in the AprilTag's
   * frame of reference.
   */
  public double getYOffset() {
    return yOffset;
  }
}
