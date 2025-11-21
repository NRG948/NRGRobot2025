/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

public class LaserCAN extends SubsystemBase {

  private static final DataLog LOG = DataLogManager.getLog();

  /** A value indicating no measurement was available on the laserCAN distance sensor. */
  public static final double NO_MEASUREMENT = 0.0;

  /** The distance between the two laser CAN devices in meters */
  private static final double DISTANCE_BETWEEN_LASER_CANS = 0.60;

  /** Amount to add to the raw distance measurements to get accurate distances. */
  private static final double LEFT_DISTANCE_CORRECTION = 0.035;

  /** Amount to add to the raw distance measurements to get accurate distances. */
  private static final double RIGHT_DISTANCE_CORRECTION = 0.025;

  /** The laser CAN closest to the funnel. */
  private LaserCan leftLaserCAN;

  /** The laser CAN closest to the ground intake. */
  private LaserCan rightLaserCAN;

  private double leftDistance = NO_MEASUREMENT;
  private double rightDistance = NO_MEASUREMENT;
  private double angleToWall = NO_MEASUREMENT;
  private boolean hasValidMeasurement = false;

  private DoubleLogEntry logLeftDistance = new DoubleLogEntry(LOG, "/LaserCAN/leftDistance");
  private DoubleLogEntry logRightDistance = new DoubleLogEntry(LOG, "/LaserCAN/rightDistance");
  private DoubleLogEntry logAngleToWall = new DoubleLogEntry(LOG, "/LaserCAN/angleToWall");

  /** Creates a new LaserCAN. */
  public LaserCAN() {
    try {
      leftLaserCAN = createLaserCAN(CAN.LEFT_LASER_CAN_ID, TIMING_BUDGET_20MS);
      rightLaserCAN = createLaserCAN(CAN.RIGHT_LASER_CAN_ID, TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public boolean hasValidMeasurement() {
    return hasValidMeasurement;
  }

  public double getAverageDistance() {
    return (leftDistance + rightDistance) / 2;
  }

  public double getLeftDistance() {
    return leftDistance;
  }

  public double getRightDistance() {
    return rightDistance;
  }

  /**
   * Returns the angle to the wall in degrees.
   *
   * @return The angle to the wall in degrees.
   */
  public double getAngleToWall() {
    return angleToWall;
  }

  private LaserCan createLaserCAN(int id, LaserCan.TimingBudget timingBudget)
      throws ConfigurationFailedException {
    LaserCan laserCAN = new LaserCan(id);
    laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
    laserCAN.setRegionOfInterest(
        new LaserCan.RegionOfInterest(8, 8, 8, 8)); // Makes detection region a box
    laserCAN.setTimingBudget(timingBudget);
    return laserCAN;
  }

  /** Updates and logs the current sensors states. */
  private void updateTelemetry() {
    leftDistance = getDistance(leftLaserCAN);
    rightDistance = getDistance(rightLaserCAN);

    if (leftDistance == NO_MEASUREMENT || rightDistance == NO_MEASUREMENT) {
      hasValidMeasurement = false;
      angleToWall = NO_MEASUREMENT;
    } else {
      hasValidMeasurement = true;
      leftDistance += LEFT_DISTANCE_CORRECTION;
      rightDistance += RIGHT_DISTANCE_CORRECTION;
      angleToWall =
          Math.toDegrees(Math.atan((leftDistance - rightDistance) / DISTANCE_BETWEEN_LASER_CANS));
    }

    logLeftDistance.append(leftDistance);
    logRightDistance.append(rightDistance);
    logAngleToWall.append(angleToWall);
  }

  private double getDistance(LaserCan laserCan) {
    if (laserCan == null) {
      return NO_MEASUREMENT;
    }

    Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm / 1000.0;
    } else {
      return NO_MEASUREMENT;
    }
  }
}
