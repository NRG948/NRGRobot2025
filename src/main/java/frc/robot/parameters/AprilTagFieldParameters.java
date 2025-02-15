/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public enum AprilTagFieldParameters {
  k2025ReefscapeWelded(AprilTagFields.k2025ReefscapeWelded),
  k2025ReefscapeAndyMark(AprilTagFields.k2025ReefscapeAndyMark);

  private AprilTagFields fieldLayout;

  private AprilTagFieldParameters(AprilTagFields AprilTagFieldLayout) {
    this.fieldLayout = AprilTagFieldLayout;
  }

  public AprilTagFields getAprilTagFieldLayout() {
    return fieldLayout;
  }
}
