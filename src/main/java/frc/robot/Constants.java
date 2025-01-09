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
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);

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
}
