/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.questnav;

import static frc.robot.Constants.Quest3S.ROBOT_TO_QUEST;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class QuestTelemetryNT implements QuestTelemetry {

  private static final float[] ZERO_VECTOR3 = new float[] {0.0f, 0.0f, 0.0f};
  private static final float[] ZERO_VECTOR4 = new float[] {0.0f, 0.0f, 0.0f, 0.0f};

  // Configure Network Tables topics (questnav/...) to communicate with the QuestNav.
  private NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
  private NetworkTable questnavTable = ntInstance.getTable("questnav");

  // Subscribe to the questnav topics in the Network Tables.
  // Reference: https://github.com/juchong/QuestNav/blob/main/unity/Assets/Robot/QuestNav.cs#L718
  private final IntegerPublisher questMosi = questnavTable.getIntegerTopic("mosi").publish();
  private final IntegerSubscriber questMiso = questnavTable.getIntegerTopic("miso").subscribe(0);
  private final FloatArraySubscriber questPosition =
      questnavTable.getFloatArrayTopic("position").subscribe(ZERO_VECTOR3);
  private final FloatArraySubscriber questQuaternion =
      questnavTable.getFloatArrayTopic("quaternion").subscribe(ZERO_VECTOR4);
  private final FloatArraySubscriber questEulerAngles =
      questnavTable.getFloatArrayTopic("eulerAngles").subscribe(ZERO_VECTOR3);
  private final DoubleSubscriber questBattery =
      questnavTable.getDoubleTopic("batteryPercent").subscribe(0.0);
  private final DoubleSubscriber questTimestamp =
      questnavTable.getDoubleTopic("timestamp").subscribe(0.0);

  /** The initial pose of the Quest3S in absolute field coordinates. */
  private Pose2d initialPose;

  /** How much to offset the raw yaw angle coming from the Quest headset (in degrees). */
  private float yawOffset = 0.0f;

  public QuestTelemetryNT() {
    setInitialPose(Pose2d.kZero);
  }

  public void updateTelemetry(QuestTelemetryData telemetry) {
    telemetry.questPose = getQuestPose();
    telemetry.robotPose = getRobotPose();
    telemetry.questTranslation = getQuestTranslation();
    telemetry.rawPose = getRawPose();

    telemetry.batteryLevel = questBattery.get();

    double timestamp = questTimestamp.get();
    telemetry.timestampDelta = timestamp - telemetry.timestamp;
    telemetry.timestamp = timestamp;

    // The timestampDelta is the change in the timestamp between the current and last robot loop.
    // The delta is zero if the new measurement is from the same timestamp as the last measurement,
    // meaning we have not received new data.
    telemetry.wasUpdated = telemetry.timestampDelta != 0;

    cleanUpOculusMessages();
  }

  /** Sets a supplied pose as the origin of all position telemetry. */
  public void setInitialPose(Pose2d pose) {
    zeroAbsolutePosition();
    initialPose = pose;
  }

  /** Zeroes the absolute 3D position of the robot (similar to long-pressing the quest logo) */
  private void zeroAbsolutePosition() {
    if (questMiso.get() != 99) {
      questMosi.set(1);
    }
  }

  private void setYawOffset() {
    yawOffset = questEulerAngles.get()[1];
  }

  /** Returns the questnav's yaw in radians with the zero-offset applied. */
  private double getYaw() {
    float yawRaw = questEulerAngles.get()[1];
    return MathUtil.angleModulus(Math.toRadians(yawRaw - yawOffset));
  }

  private Translation2d getQuestTranslation() {
    float[] oculusPosition = questPosition.get();
    // return new Translation2d(-oculusPosition[0], oculusPosition[2]);
    return new Translation2d(oculusPosition[2], -oculusPosition[0]);
  }

  private Pose2d getRawPose() {
    return new Pose2d(getQuestTranslation(), Rotation2d.fromRadians(getYaw()));
  }

  private Pose2d getQuestPose() {
    return new Pose2d(
        getQuestTranslation().minus(ROBOT_TO_QUEST), Rotation2d.fromRadians(getYaw()));
  }

  private Pose2d getRobotPose() {
    return new Pose2d(
        getQuestPose().minus(initialPose).getTranslation(), Rotation2d.fromRadians(getYaw()));
  }

  /**
   * IMPORTANT: Run periodically after processing.<br>
   * Cleans up Oculus subroutine messages after processing on the headset.
   */
  private void cleanUpOculusMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  @Override
  public void close() {
    questMiso.close();
    questMosi.close();
    questTimestamp.close();
    questPosition.close();
    questQuaternion.close();
    questEulerAngles.close();
    questBattery.close();
  }
}
