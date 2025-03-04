/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import static frc.robot.Constants.VisionConstants.BRANCH_TO_REEF_APRILTAG;

/**
 * An enum that represents the reef alignment positions as viewed face on.
 *
 * @param yOffset The y offset of the reef position from the center reef position directly above the
 *     center of the AprilTag.
 */
public enum ReefPosition {
  LEFT_BRANCH(-BRANCH_TO_REEF_APRILTAG),
  CENTER_REEF(0.0),
  RIGHT_BRANCH(BRANCH_TO_REEF_APRILTAG);

  private double yOffset;

  ReefPosition(double yOffset) {
    this.yOffset = yOffset;
  }

  public double yOffset() {
    return yOffset;
  }
}
