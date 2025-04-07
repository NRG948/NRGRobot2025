/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS;
import static au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget.TIMING_BUDGET_33MS;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

@RobotPreferencesLayout(
    groupName = "LaserCAN",
    row = 4,
    column = 0,
    width = 2,
    height = 1,
    type = "Grid Layout",
    gridColumns = 2,
    gridRows = 2)
public class LaserCAN extends SubsystemBase implements ShuffleboardProducer {

  @RobotPreferencesValue(column = 0, row = 0)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("LaserCAN", "Enable Tab", false);

  private static final DataLog LOG = DataLogManager.getLog();

  /** A value indicating no measurement was available on the laserCAN distance sensor. */
  private static final double NO_MEASUREMENT = 0.0;

  /** The minimum detection distance from the laserCAN to the coral station wall. */
  private static final double CORAL_STATION_MIN_DISTANCE = Units.inchesToMeters(8.0);

  /** The maximum detection distance from the laserCAN to the coral station wall. */
  private static final double CORAL_STATION_MAX_DISTANCE =
      CORAL_STATION_MIN_DISTANCE + Units.inchesToMeters(13.0);

  /** The laser CAN closest to the funnel. */
  private LaserCan leftLaserCAN;

  /** The laser CAN closest to the ground intake. */
  private LaserCan rightLaserCAN;

  private boolean detectsWall = false;
  private boolean leftLaserCANDetected = false;
  private boolean rightLaserCANDetected = false;

  private double leftDistance = NO_MEASUREMENT;
  private double rightDistance = NO_MEASUREMENT;

  private DoubleLogEntry logLeftDistance = new DoubleLogEntry(LOG, "/LaserCAN/leftDistance");
  private DoubleLogEntry logRightDistance = new DoubleLogEntry(LOG, "/LaserCAN/rightDistance");
  private BooleanLogEntry logDetectsWall = new BooleanLogEntry(LOG, "/LaserCAN/detectsWall");

  /** Creates a new LaserCAN. */
  public LaserCAN() {
    try {
      leftLaserCAN = createLaserCAN(CAN.LEFT_LASER_CAN_ID, TIMING_BUDGET_20MS);
      rightLaserCAN = createLaserCAN(CAN.RIGHT_LASER_CAN_ID, TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public boolean detectsWall() {
    return detectsWall;
  }

  public boolean isLeftLaserCANDetected() {
    return leftLaserCANDetected;
  }

  public boolean isRightLaserCANDetected() {
    return rightLaserCANDetected;
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
        new LaserCan.RegionOfInterest(8, 8, 16, 4)); // Makes detection region a box
    laserCAN.setTimingBudget(timingBudget);
    return laserCAN;
  }

  /** Updates and logs the current sensors states. */
  private void updateTelemetry() {
    leftDistance = getDistance(leftLaserCAN);
    rightDistance = getDistance(rightLaserCAN);

    leftLaserCANDetected =
        leftDistance >= CORAL_STATION_MIN_DISTANCE && leftDistance <= CORAL_STATION_MAX_DISTANCE;
    rightLaserCANDetected =
        rightDistance >= CORAL_STATION_MIN_DISTANCE && rightDistance <= CORAL_STATION_MAX_DISTANCE;
    detectsWall = leftLaserCANDetected && rightLaserCANDetected;

    logLeftDistance.append(leftDistance);
    logRightDistance.append(rightDistance);
    logDetectsWall.append(detectsWall);
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
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab LaserCANTab = Shuffleboard.getTab("Laser CAN");
    ShuffleboardLayout statusLayout =
        LaserCANTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 8);
    statusLayout.addBoolean("Detects Wall", () -> detectsWall);
    statusLayout.addBoolean("Detects Right LaserCAN", () -> rightLaserCANDetected);
    statusLayout.addBoolean("Detects Left LaserCAN", () -> leftLaserCANDetected);
    statusLayout.addDouble("Left Distance", () -> leftDistance);
    statusLayout.addDouble("Right Distance", () -> leftDistance);
  }
}
