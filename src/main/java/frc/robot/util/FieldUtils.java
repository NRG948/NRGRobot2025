/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;

/** Helper methods related to the 2025 FRC ReefScape field. */
public final class FieldUtils {

  // TODO: Make field layout selectable.
  private static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final ArrayList<Pose2d> redReefTags = new ArrayList<>(6);
  private static final ArrayList<Pose2d> blueReefTags = new ArrayList<>(6);

  private static final ArrayList<Pose2d> redCoralStationTags = new ArrayList<>(2);
  private static final ArrayList<Pose2d> blueCoralStationTags = new ArrayList<>(2);

  static {
    // Red reef Apriltags are IDs 6-11; Blue reef Apriltags are IDs 17-22.
    for (int i = 0; i < 6; i++) {
      redReefTags.add(getAprilTagPose2d(i + 6));
      blueReefTags.add(getAprilTagPose2d(i + 17));
    }

    for (int i = 0; i < 2; i++) {
      redCoralStationTags.add(getAprilTagPose2d(i + 1));
      blueCoralStationTags.add(getAprilTagPose2d(i + 12));
    }
  }

  private FieldUtils() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /** Returns true if we are on the Red alliance. Defaults to Blue if alliance is not set. */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Red;
  }

  /** Returns the {@link AprilTagFieldLayout} for the current competition year. */
  public static AprilTagFieldLayout getFieldLayout() {
    return FIELD_LAYOUT;
  }

  /** Returns the {@link Pose3d} of the specified April Tag ID. */
  public static Pose3d getAprilTagPose3d(int tagId) {
    return FIELD_LAYOUT.getTagPose(tagId).get();
  }

  /** Returns the {@link Pose2d} of the specified April Tag ID. */
  public static Pose2d getAprilTagPose2d(int tagId) {
    return getAprilTagPose3d(tagId).toPose2d();
  }

  /** Returns a list of AprilTag poses for our alliance's reef. */
  public static ArrayList<Pose2d> getReefAprilTags() {
    return isRedAlliance() ? redReefTags : blueReefTags;
  }

  /** Returns a list of AprilTag poses for our alliance's coral stations. */
  public static ArrayList<Pose2d> getCoralStationAprilTags() {
    return isRedAlliance() ? redCoralStationTags : blueCoralStationTags;
  }
}
