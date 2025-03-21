/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.questnav;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

  private Transform2d questNavToRobot;

  /** How much to offset the raw yaw angle coming from the Quest headset (in degrees). */
  private float yawOffset = 0.0f;

  public QuestTelemetryNT() {
    setInitialPose(Pose2d.kZero);
  }

  public void updateTelemetry(QuestTelemetryData telemetry) {
    cleanUpOculusMessages();

    double timestamp = questTimestamp.get();
    double timestampDelta = timestamp - telemetry.timestamp;
    telemetry.timestamp = timestamp;

    // The timestampDelta is the change in the timestamp between the current and last robot loop.
    // The delta is zero if the new measurement is from the same timestamp as the last measurement,
    // meaning we have not received new data.
    telemetry.wasUpdated = timestampDelta != 0;

    if (telemetry.wasUpdated) {

      telemetry.questPose = getQuestPose();
      telemetry.robotPose = getRobotPose();

      telemetry.batteryLevel = questBattery.get();
    }
  }

  public void setQuestNavToRobotTransform(Transform2d questNavToRobot) {
    this.questNavToRobot = questNavToRobot;
  }

  public boolean ping() {
    if (questMiso.get() != 97) {
      questMosi.set(3);
      return false;
    }
    return true;
  }

  /** Sets a supplied pose as the origin of all position telemetry. */
  public void setInitialPose(Pose2d pose) {
    initialPose = pose;
  }

  /** Zeroes the absolute 3D position of the QuestNav (similar to long-pressing the quest logo) */
  public boolean zeroPose() {
    if (questMiso.get() != 98) {
      questMosi.set(2);
      return false;
    }
    return true;
  }

  /** Zeroes the heading of the Questnav (similar to long-pressing the quest logo) */
  public boolean zeroHeading() {
    if (questMiso.get() != 99) {
      questMosi.set(1);
      return false;
    }
    return true;
  }

  /** Returns the questnav's yaw in radians with the zero-offset applied. */
  private double getYaw() {
    float yawRaw = questEulerAngles.get()[1];
    return MathUtil.angleModulus(-Math.toRadians(yawRaw - yawOffset));
  }

  private Translation2d getQuestTranslation() {
    float[] oculusPosition = questPosition.get();
    return new Translation2d(-oculusPosition[0], -oculusPosition[2]);
  }

  private Pose2d getQuestPose() {
    return new Pose2d(getQuestTranslation(), Rotation2d.fromRadians(getYaw()))
        .relativeTo(initialPose);
  }

  private Pose2d getRobotPose() {
    return getQuestPose().plus(questNavToRobot);
  }

  /**
   * IMPORTANT: Run periodically after processing.<br>
   * Cleans up Oculus subroutine messages after processing on the headset.
   */
  private void cleanUpOculusMessages() {
    if (questMiso.get() != 0) {
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
