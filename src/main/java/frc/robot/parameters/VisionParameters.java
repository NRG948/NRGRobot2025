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
import java.util.Optional;

/** Add your docs here. */
public enum VisionParameters {
  CompetitionBase2024(
      Optional.of(
          new Transform3d(
              new Translation3d(Units.inchesToMeters(11.375), 0, Units.inchesToMeters(23.625)),
              new Rotation3d(0, Math.toRadians(21.5), 0))),
      Optional.empty()),
  PracticeBase2025(
      Optional.of(
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(11),
                  Units.inchesToMeters(-10.25),
                  Units.inchesToMeters(12.625)),
              new Rotation3d())),
      Optional.of(
          new Transform3d(
              new Translation3d(Units.inchesToMeters(-13), -0.255, -0.8175),
              new Rotation3d(0, 0, Math.toRadians(180)))));

  private final Optional<Transform3d> frontCamera;
  private final Optional<Transform3d> backCamera;

  /**
   * The constructor for robot vision parameters.
   *
   * @param frontCamera The transform of the front camera from the center of the robot.
   * @param backCamera The transform of the back camera from the center of the robot.
   */
  private VisionParameters(Optional<Transform3d> frontCamera, Optional<Transform3d> backCamera) {
    this.frontCamera = frontCamera;
    this.backCamera = backCamera;
  }

  /** Returns the transform of the front camera from the center of the robot. */
  public Optional<Transform3d> getRobotToFrontCamera() {
    return frontCamera;
  }

  /** Returns the transform of the back camera from the center of the robot. */
  public Optional<Transform3d> getRobotToBackCamera() {
    return backCamera;
  }
}
