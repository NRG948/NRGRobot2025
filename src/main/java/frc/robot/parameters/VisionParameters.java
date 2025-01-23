/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public enum VisionParameters {
  PracticeBase2024(0, 0, 0, 0, 0, 180), // TODO: Get real values
  CompetitionBase2024(
      11.375, 0, 23.625, 0, -21.5,
      0); // test robot, flipping front/back of the robot compared to 2024

  private double x;
  private double y;
  private double z;
  private double roll;
  private double pitch;
  private double yaw;

  /**
   * The constructor for robot vision parameters.
   *
   * @param x The x offset in inches.
   * @param y The y offset in inches.
   * @param z The z offset in inches.
   * @param roll The roll in degrees.
   * @param pitch The pitch in degrees.
   * @param yaw The yaw in degrees.
   */
  private VisionParameters(double x, double y, double z, double roll, double pitch, double yaw) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.roll = roll;
    this.pitch = pitch;
    this.yaw = yaw;
  }

  public Transform3d getRobotToCamera() {
    return new Transform3d(
        new Translation3d(
            Units.inchesToMeters(x), Units.inchesToMeters(y), Units.inchesToMeters(z)),
        new Rotation3d(Math.toRadians(roll), Math.toRadians(pitch), Math.toRadians(yaw)));
  }
}
