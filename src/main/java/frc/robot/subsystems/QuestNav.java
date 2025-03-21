/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.Quest3S.QUEST_TO_ROBOT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.questnav.QuestTelemetry;
import frc.robot.questnav.QuestTelemetry.QuestTelemetryData;
import frc.robot.questnav.QuestTelemetryNT;

public class QuestNav extends SubsystemBase {

  private enum State {
    INITIALIZING,
    RESETTING_POSE,
    READY;
  }

  QuestTelemetry telemetry = new QuestTelemetryNT();
  QuestTelemetryData telemetryData = new QuestTelemetryData();

  State state = State.INITIALIZING;
  Pose2d initialRobotPose;

  /** Creates a new QuestNav. */
  public QuestNav() {}

  @Override
  public void periodic() {
    switch (state) {
      case INITIALIZING:
        if (telemetry.ping()) {
          state = State.RESETTING_POSE;
        }
        break;
      case RESETTING_POSE:
        if (telemetry.zeroPose()) {
          state = State.READY;
          telemetry.setInitialPose(initialRobotPose.plus(QUEST_TO_ROBOT.inverse()));
          telemetry.setQuestNavToRobotTransform(QUEST_TO_ROBOT);
        }
        break;
      case READY:
        telemetry.updateTelemetry(telemetryData);
      default:
        break;
    }
  }

  public void setInitialRobotPose(Pose2d initialRobotPose) {
    this.initialRobotPose = initialRobotPose;
  }

  public QuestTelemetryData getQuestTelemetryData() {
    return telemetryData;
  }
}
