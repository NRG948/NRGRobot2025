/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

// import org.littletonrobotics.junction.AutoLog; // TODO: investigate this package

public interface QuestTelemetry extends AutoCloseable {

  // @Autolog
  public static class QuestTelemetryData {
    /** Is the QuestNav headset connected to the network. */
    public boolean wasUpdated = false;

    /** The QuestNav's battery level. */
    public double batteryLevel = 0;

    /** The timestamp of the last telemetry update from the QuestNav. */
    public double timestamp = 0;

    /** */
    public double timestampDelta = 0;

    /** The field-relative pose of the QuestNav device (i.e. offset by the initial pose). */
    public Pose2d questPose = new Pose2d();

    /** The field-relative pose of the robot (i.e. offset by the initial pose). */
    public Pose2d robotPose = new Pose2d();

    public Translation2d questTranslation = new Translation2d();

    public Pose2d rawPose = new Pose2d();
  }

  /** Refreshes the latest questnav telemetry from the Network Tables. */
  public default void updateTelemetry(QuestTelemetryData telemetry) {}

  /** Sets supplied pose as the origin of all position telemetry. */
  public default void setInitialPose(Pose2d pose) {}

  @Override
  public default void close() {}
}
