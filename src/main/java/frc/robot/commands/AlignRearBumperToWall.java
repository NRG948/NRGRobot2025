/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences.DoubleValue;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaserCAN;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

@RobotPreferencesLayout(
    groupName = "AlignRearBumperToWall",
    row = 0,
    column = 8,
    width = 2,
    height = 2)
public class AlignRearBumperToWall extends Command {
  private static final DataLog LOG = DataLogManager.getLog();
  private static final double MAX_TRANSLATIONAL_POWER = 0.6;
  private static final double MAX_ROTATIONAL_POWER = 0.5;

  /** The translational tolerance value for aligning to the wall. */
  public static final double ALIGNMENT_TOLERANCE_X = 0.015; // in m

  /** The rotational tolerance value for aligning to the wall. */
  public static final double ALIGNMENT_TOLERANCE_R = 2.0; // in deg

  /** The x distance from the laser CANs to the edge of the rear bumper. */
  public static final double LASER_CAN_TO_REAR_BUMBER_DELTA_X = 0.13;

  @RobotPreferencesValue
  public static DoubleValue Px = new DoubleValue("AlignRearBumperToWall", "X KP", 1);

  @RobotPreferencesValue
  public static DoubleValue Pr = new DoubleValue("AlignRearBumperToWall", "R KP", 0.02);

  private final Swerve drivetrain;
  private final LaserCAN laserCAN;

  private final PIDController xController = new PIDController(Px.getValue(), 0, 0);
  private final PIDController rController = new PIDController(Pr.getValue(), 0, 0);

  private final DoubleLogEntry xErrorLog = new DoubleLogEntry(LOG, "AlignToWall/X Error");
  private final DoubleLogEntry rErrorLog = new DoubleLogEntry(LOG, "AlignToWall/Yaw Error");

  private final Timer timer = new Timer();

  private double xTarget = LASER_CAN_TO_REAR_BUMBER_DELTA_X;
  private double rTarget = 0.0; // in degrees

  /** Creates a new AlignRearBumperToWall. */
  public AlignRearBumperToWall(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain;
    this.laserCAN = subsystems.laserCAN;

    addRequirements(drivetrain, laserCAN);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set up the PID controllers to drive to the target pose.
    xController.setSetpoint(xTarget);
    rController.setSetpoint(rTarget);

    xController.setPID(Px.getValue(), 0, 0);
    rController.setPID(Pr.getValue(), 0, 0);

    xController.setTolerance(ALIGNMENT_TOLERANCE_X);
    rController.setTolerance(ALIGNMENT_TOLERANCE_R);

    rController.enableContinuousInput(-180, 180);

    timer.stop();
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentX = laserCAN.getAverageDistance();
    double currentR = laserCAN.getAngleToWall();

    double xError = xTarget - currentX;
    double rError = rTarget - currentR;

    xErrorLog.append(xError);
    rErrorLog.append(rError);

    double xSpeed = 0;
    double rSpeed = 0;
    if (laserCAN.hasValidMeasurement()) {
      xSpeed =
          MathUtil.clamp(
              xController.calculate(currentX), -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);
      if (!(Math.abs(rError) < ALIGNMENT_TOLERANCE_R)) {
        rSpeed =
            MathUtil.clamp(
                rController.calculate(currentR), -MAX_ROTATIONAL_POWER, MAX_ROTATIONAL_POWER);
      }
    }

    drivetrain.drive(xSpeed, 0, rSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (xController.atSetpoint() && rController.atSetpoint()) {
      timer.start();
    } else {
      timer.reset();
    }

    return timer.hasElapsed(0.1);
  }
}
