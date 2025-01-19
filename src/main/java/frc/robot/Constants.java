/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.parameters.VisionParameters;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /** Constants for robot elements. */
  public static class RobotConstants {
    /** The maximum battery voltage. */
    public static final double MAX_BATTERY_VOLTAGE = 12.0;

    /** The swerve drive wheel diameter. */
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

    /** CANBus device IDs. */
    public static final class CAN {
      /** The pigeon2 ID. */
      public static final int PIGEON_ID = 1;
    }

    /** Digital I/O port numbers. */
    public static class DigitalIO {
      public static final int ARM_ABSOLUTE_ENCODER = 0;
    }
  }

  /** Constants for the robot operator. */
  public static class OperatorConstants {

    /** The driver Xbox controller port. */
    public static final int DRIVER_CONTROLLER_PORT = 0;

    /** The driver Xbox manipulator port. */
    public static final int MANIPULATOR_CONTROLLER_PORT = 1;
  }

  public static final String CAMERA1_NAME = "948Mono001";

  @RobotPreferencesValue
  public static RobotPreferences.EnumValue<VisionParameters> PARAMETERS =
      new RobotPreferences.EnumValue<VisionParameters>(
          "AprilTag", "Robot Vision", VisionParameters.CompetitionBase2024);

  /** Used to transform the center of robot position to the camera position. */
  public static final Transform3d APRILTAG_ROBOT_TO_CAMERA1 =
      PARAMETERS.getValue().getRobotToCamera();

  /** Used to transform the camera position to the center of robot position. */
  public static final Transform3d APRILTAG_CAMERA_TO_ROBOT = APRILTAG_ROBOT_TO_CAMERA1.inverse();

  /** Field of view of the AprilTag camera in degrees. */
  public static final double APRILTAG_CAMERA_FOV = 70;
}
