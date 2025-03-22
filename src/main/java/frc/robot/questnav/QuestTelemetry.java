/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

/** An interface for managing QuestNav telemetry. */
public interface QuestTelemetry extends AutoCloseable {

  /** A class containing QuestNav telemetry. */
  public static class QuestTelemetryData {
    /** Is the QuestNav headset connected to the network. */
    public boolean wasUpdated = false;

    /** The QuestNav's battery level. */
    public double batteryLevel = 0;

    /** The timestamp of the last telemetry update from the QuestNav. */
    public double timestamp = 0;

    /** The field-relative pose of the robot (i.e. offset by the initial pose). */
    public Pose2d robotPose = new Pose2d();

    /** The field-relative pose of the QuestNav (i.e. offset by the initial pose). */
    public Pose2d questPose = new Pose2d();
  }

  /**
   * Sets the transform mapping the QuestNav to the center of the robot.
   *
   * @param questNavToRobot
   */
  public default void setQuestNavToRobotTransform(Transform2d questNavToRobot) {}

  /** Refreshes the latest questnav telemetry from the Network Tables. */
  public default void updateTelemetry(QuestTelemetryData telemetry) {}

  /**
   * Pings the QuestNav for readiness.
   *
   * @return Returns true when the QuestNav is ready.
   */
  public default boolean ping() {
    return true;
  }

  /** Sets supplied pose as the origin of all position telemetry. */
  public default void setInitialQuestPose(Pose2d pose) {}

  /**
   * Zeroes the pose of the QuestNav.
   *
   * @return Returns true when completed.
   */
  public default boolean zeroPose() {
    return true;
  }

  /**
   * Zeroes the heading of the QuestNav.
   *
   * @return Returns true when completed.
   */
  public default boolean zeroHeading() {
    return true;
  }

  @Override
  public default void close() {}
}
