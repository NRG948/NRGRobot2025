/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import com.nrg948.preferences.RobotPreferences;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * AprilTag field layout selections for REEFSCAPE.
 *
 * <p>The PROCESSOR AprilTag location varies depending on the field perimeter in use. See <a
 * href="https://firstfrc.blob.core.windows.net/frc2025/Manual/TeamUpdates/TeamUpdate12.pdf">Team
 * Update 12</a> for more information.
 *
 * <p>This enum can be used with a {@link SendableChooser} or {@link RobotPreferences.EnumValue} to
 * select the field layout to match.
 */
public enum AprilTagFieldParameters {
  /** The field layout using the welded perimeter. */
  k2025ReefscapeWelded(AprilTagFields.k2025ReefscapeWelded),

  /** The field layout using the AndyMark perimeter. */
  k2025ReefscapeAndyMark(AprilTagFields.k2025ReefscapeAndyMark);

  private AprilTagFields fieldLayout;

  /** Constructs a variant of this enum. */
  private AprilTagFieldParameters(AprilTagFields AprilTagFieldLayout) {
    this.fieldLayout = AprilTagFieldLayout;
  }

  /** Gets the {@link AprilTagFields} variant that can be used to load the correct field layout. */
  public AprilTagFields getAprilTagField() {
    return fieldLayout;
  }

  /** Gets the {@link AprilTagFieldLayout} for this variant. */
  public AprilTagFieldLayout loadAprilTagFieldLayout() {
    return AprilTagFieldLayout.loadField(fieldLayout);
  }
}
