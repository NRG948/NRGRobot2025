/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import edu.wpi.first.math.util.Units;

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
    public static final double WHEEL_DIAMETER =
        Units.inchesToMeters(4.0); // WARNING: need to be 3.625 for crescendo bot!

    /** The length of the robot including bumpers. */
    // TODO: GET REAL VALUE
    public static final double ROBOT_LENGTH = 0.928;

    /** The width of the robot including bumpers. */
    // TODO: GET REAL VALUE
    public static final double ROBOT_WIDTH = 0.9;

    /**
     * The x distance from the odometry center (center of the wheels) to the edge of the front
     * bumper.
     */
    public static final double ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X = ROBOT_LENGTH / 2;

    /**
     * The x distance from the odometry center (center of the wheels) to the edge of the rear
     * bumper.
     */
    public static final double ODOMETRY_CENTER_TO_REAR_BUMPER_DELTA_X = ROBOT_LENGTH / 2;

    /** The robot iteration period in seconds. */
    public static final double PERIODIC_INTERVAL = 0.02;

    /**
     * The y-offset (in the width direction) of the coral end effector from the center of the robot
     */
    public static final double CORAL_OFFSET_Y = 0.20; // TODO: GET REAL VALUE

    public static class PWMPort {
      public static final int LED = 1;
    }

    public static class LEDSegment {
      public static final int STATUS_FIRST_LED = 0;
      public static final int STATUS_LED_COUNT = 56; // 33 on front, 23 on back
    }

    public static final int LED_COUNT = 56;

    /** CANBus device IDs. */
    public static final class CAN {
      public static final class TalonFX {
        public static final int ELEVATOR_MAIN_MOTOR_ID = 14; // Back
        public static final int ELEVATOR_FOLLOWER_MOTOR_ID = 8; // Front
        public static final int CORAL_ARM_MOTOR_ID = 6; // TODO check with systems for final IDs.
        public static final int CORAL_ROLLER_MOTOR_ID = 5;
        public static final int ALGAE_ARM_MOTOR_ID = 3;
        public static final int ALGAE_GRABBER_MOTOR_ID = 4;
        public static final int CLIMBER_MOTOR_ID = 11;
      }
    }

    /** Digital I/O port numbers. */
    public static class DigitalIO {
      public static final int CORAL_ROLLER_BEAM_BREAK = 9;
      public static final int CORAL_ARM_ABSOLUTE_ENCODER = 0;
      public static final int ALGAE_ARM_ABSOLUTE_ENCODER = 2;
      public static final int CLIMBER_ABSOLUTE_ENCODER = 3;
    }
  }

  /** Constants for the robot operator. */
  public static class OperatorConstants {

    /** The driver Xbox controller port. */
    public static final int DRIVER_CONTROLLER_PORT = 0;

    /** The driver Xbox manipulator port. */
    public static final int MANIPULATOR_CONTROLLER_PORT = 1;
  }

  public static class VisionConstants {

    /** The distance from each reef branch from the center of the apriltag. */
    public static final double BRANCH_TO_REEF_APRILTAG = 0.165;

    /** The translational tolerance value for aligning to the reef. */
    public static final double POSE_ALIGNMENT_TOLERANCE_XY = 0.0333; // in m

    /** The rotational tolerance value for aligning to the reef. */
    public static final double POSE_ALIGNMENT_TOLERANCE_R = 1.0; // in deg
  }
}
