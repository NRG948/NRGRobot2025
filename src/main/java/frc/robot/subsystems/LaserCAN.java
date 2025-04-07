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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

public class LaserCAN extends SubsystemBase implements ShuffleboardProducer {

  private static final DataLog LOG = DataLogManager.getLog();

  /** A value indicating no measurement was available on the laserCAN distance sensor. */
  private static final double NO_MEASUREMENT = 0.0;

  /** The laser CAN closest to the funnel. */
  private LaserCan leftLaserCAN;

  /** The laser CAN closest to the ground intake. */
  private LaserCan rightLaserCAN;

  private double leftDistance = NO_MEASUREMENT;
  private double rightDistance = NO_MEASUREMENT;

  private DoubleLogEntry logLeftDistance = new DoubleLogEntry(LOG, "/LaserCAN/leftDistance");
  private DoubleLogEntry logRightDistance = new DoubleLogEntry(LOG, "/LaserCAN/rightDistance");

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

  public double getAverageDistance() {
    return (leftDistance + rightDistance) / 2;
  }

  public double getLeftDistance() {
    return leftDistance;
  }

  public double getRightDistance() {
    return rightDistance;
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

    logLeftDistance.append(leftDistance);
    logRightDistance.append(rightDistance);
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

  @Override
  public void addShuffleboardTab() {
    ShuffleboardTab LaserCANTab = Shuffleboard.getTab("Laser CAN");
    ShuffleboardLayout statusLayout =
        LaserCANTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 8);
    statusLayout.addDouble("Left Distance", () -> leftDistance);
    statusLayout.addDouble("Right Distance", () -> rightDistance);
    statusLayout.addDouble("Average Distance", this::getAverageDistance);
  }
}
