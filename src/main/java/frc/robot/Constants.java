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
import edu.wpi.first.wpilibj.util.Color8Bit;
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

    public static class PWNPort {
      public static final int LED = 1; // TODO: change value
    }

    public static class LEDSegment {
      public static final int STATUS_FIRST_LED = 0; // TODO: change value
      public static final int STATUS_LED_COUNT = 26; // TODO: change value
    }   

    public static final int LED_COUNT = 1; // TODO: change value

    /** CANBus device IDs. */
    public static final class CAN {
      /** The pigeon2 ID. */
      public static final int PIGEON_ID = 1;
    }
  }

  /** Constants for the robot operator. */
  public static class OperatorConstants {
    /** The driver Xbox controller port. */
    public static final int DRIVER_CONTROLLER_PORT = 0;
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

  public static class ColorConstants {
    public static final Color8Bit BLACK = new Color8Bit(0, 0, 0);
    public static final Color8Bit WHITE = new Color8Bit(200, 200, 200);
    public static final Color8Bit RED = new Color8Bit(255, 0, 0);
    public static final Color8Bit ORANGE = new Color8Bit(255, 119, 0);
    public static final Color8Bit YELLOW = new Color8Bit(255, 165, 0);
    public static final Color8Bit GREEN = new Color8Bit(0, 204, 0);
    public static final Color8Bit BLUE = new Color8Bit(0, 0, 204);
    public static final Color8Bit PURPLE = new Color8Bit(238, 80, 255);
    public static final Color8Bit PINK = new Color8Bit(255, 5, 100);
    public static final Color8Bit LIGHTBLUE = new Color8Bit(56, 197, 252);

    public static final Color8Bit COLORS[] = {
          BLACK, WHITE, RED, ORANGE, YELLOW, GREEN, BLUE, LIGHTBLUE, PURPLE, PINK,
        };
  }
}
