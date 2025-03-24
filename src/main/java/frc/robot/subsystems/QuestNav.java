/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.Quest3S.QUEST_TO_ROBOT;
import static frc.robot.Constants.Quest3S.ROBOT_TO_QUEST;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.questnav.QuestTelemetry;
import frc.robot.questnav.QuestTelemetry.QuestTelemetryData;
import frc.robot.questnav.QuestTelemetryNT;

public class QuestNav extends SubsystemBase implements ShuffleboardProducer {

  private enum State {
    INITIALIZING,
    RESETTING_POSE,
    READY;
  }

  private static final DataLog LOG = DataLogManager.getLog();

  private final QuestTelemetryData telemetryData = new QuestTelemetryData();
  private final QuestTelemetry telemetry = new QuestTelemetryNT(telemetryData);

  private State state = State.INITIALIZING;
  private Pose2d initialRobotPose = Pose2d.kZero;

  private final StructLogEntry<Pose2d> logRobotPose =
      StructLogEntry.create(LOG, "/QuestNav/RobotPose", Pose2d.struct);
  private final StructLogEntry<Pose2d> logRawQuestPose =
      StructLogEntry.create(LOG, "/QuestNav/RawQuestPose", Pose2d.struct);
  private final StructLogEntry<Pose2d> logQuestPose =
      StructLogEntry.create(LOG, "/QuestNav/QuestPose", Pose2d.struct);
  private final BooleanLogEntry logWasUpdated = new BooleanLogEntry(LOG, "/QuestNav/WasUpdated");
  private final StringLogEntry logState = new StringLogEntry(LOG, "/QuestNav/State");

  /** Creates a new QuestNav. */
  public QuestNav() {}

  @Override
  public void periodic() {
    logState.update(state.name());
    switch (state) {
      case INITIALIZING:
        if (telemetry.ping()) {
          state = State.RESETTING_POSE;
        }
        break;
      case RESETTING_POSE:
        state = State.READY;
        telemetry.setInitialQuestFieldPose(initialRobotPose.plus(ROBOT_TO_QUEST));
        telemetry.setQuestNavToRobotTransform(QUEST_TO_ROBOT);
        break;
      case READY:
        telemetry.updateTelemetry(telemetryData);
        logRawQuestPose.append(telemetryData.rawQuestPose);
        logQuestPose.append(telemetryData.questPose);
        logRobotPose.append(telemetryData.robotPose);
        logWasUpdated.update(telemetryData.wasUpdated);
        break;
    }
  }

  public void setInitialRobotPose(Pose2d initialRobotPose) {
    this.initialRobotPose = initialRobotPose;
    if (state == State.READY) {
      state = State.RESETTING_POSE;
    }
  }

  public void resetOrientation(Rotation2d orientation) {
    telemetry.resetOrientation(orientation);
  }

  public QuestTelemetryData getQuestTelemetryData() {
    return telemetryData;
  }

  public double getBatteryPercent() {
    return telemetryData.batteryLevel;
  }

  public Pose2d getRobotPose() {
    return telemetryData.robotPose;
  }

  public boolean hasData() {
    return telemetryData.wasUpdated;
  }

  @Override
  public void addShuffleboardTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("QuestNav");

    ShuffleboardLayout statusLayout =
        tab.getLayout("Status", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    statusLayout.addBoolean("Has Data", this::hasData);
    statusLayout.addDouble("Battery Percent", this::getBatteryPercent);
    statusLayout.addString("State", () -> state.name());

    ShuffleboardLayout poseLayout =
        tab.getLayout("Robot Pose", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
    poseLayout.addDouble("Pose X", () -> this.getRobotPose().getTranslation().getX());
    poseLayout.addDouble("Pose Y", () -> this.getRobotPose().getTranslation().getY());
    poseLayout.addDouble("Angle", () -> this.getRobotPose().getRotation().getDegrees());

    ShuffleboardLayout controlLayout =
        tab.getLayout("Control", BuiltInLayouts.kList).withPosition(4, 0).withSize(2, 4);
    // controlLayout.add(Commands.runOnce(() -> zeroPosition(), this).withName("Zero Position"));
    // controlLayout.add(Commands.runOnce(() -> zeroHeading(), this).withName("Zero Heading"));
  }
}
