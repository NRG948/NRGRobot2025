/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.parameters.AprilTagFieldParameters;
import java.util.ArrayList;

/** Helper methods related to the 2025 FRC ReefScape field. */
public final class FieldUtils {
  private static final int REEF_APRILTAG_COUNT = 6;
  private static final int FIRST_RED_REEF_APRILTAG_ID = 6;
  private static final int FIRST_BLUE_REEF_APRILTAG_ID = 17;

  private static final int CORAL_STATION_APRIL_TAG_COUNT = 2;
  private static final int FIRST_RED_CORAL_STATION_APRILTAG_ID = 1;
  private static final int FIRST_BLUE_CORAL_STATION_APRILTAG_ID = 12;

  @RobotPreferencesValue(column = 1, row = 1)
  public static RobotPreferences.EnumValue<AprilTagFieldParameters> FIELD_LAYOUT_PREFERENCE =
      new RobotPreferences.EnumValue<AprilTagFieldParameters>(
          "AprilTag", "Field Layout", AprilTagFieldParameters.k2025ReefscapeWelded);

  private static AprilTagFieldLayout FIELD_LAYOUT =
      FIELD_LAYOUT_PREFERENCE.getValue().loadAprilTagFieldLayout();

  private static final ArrayList<Pose2d> redReefTags = new ArrayList<>(6);
  private static final ArrayList<Pose2d> blueReefTags = new ArrayList<>(6);

  private static final ArrayList<Pose2d> redCoralStationTags = new ArrayList<>(2);
  private static final ArrayList<Pose2d> blueCoralStationTags = new ArrayList<>(2);

  static {
    // Red reef AprilTags are IDs 6-11; Blue reef AprilTags are IDs 17-22.
    for (int i = 0; i < REEF_APRILTAG_COUNT; i++) {
      redReefTags.add(getAprilTagPose2d(i + FIRST_RED_REEF_APRILTAG_ID));
      blueReefTags.add(getAprilTagPose2d(i + FIRST_BLUE_REEF_APRILTAG_ID));
    }

    // Red coral station AprilTags are IDs 1-2; Blue coral station AprilTags are IDs 12-13.
    for (int i = 0; i < CORAL_STATION_APRIL_TAG_COUNT; i++) {
      redCoralStationTags.add(getAprilTagPose2d(i + FIRST_RED_CORAL_STATION_APRILTAG_ID));
      blueCoralStationTags.add(getAprilTagPose2d(i + FIRST_BLUE_CORAL_STATION_APRILTAG_ID));
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

  /**
   * Returns the {@link Pose2d} describing the robot's expected pose at the specified position of
   * the nearest reef side.
   *
   * @param currentRobotPose The swerve drivetrain.
   * @param reefPosition The target reef position.
   * @return The {@link Pose2d} of the nearest reef AprilTag to the robot's current position.
   */
  public static Pose2d getRobotPoseForNearestReefAprilTag(
      Pose2d currentRobotPose, ReefPosition reefPosition) {
    Pose2d nearestTagPose = currentRobotPose.nearest(getReefAprilTags());

    return nearestTagPose.plus(reefPosition.tagToRobot());
  }
}
