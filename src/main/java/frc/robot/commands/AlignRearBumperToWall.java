/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.Constants.RobotConstants.LASER_CAN_TO_REAR_BUMBER_DELTA_X;

import com.nrg948.preferences.RobotPreferences.DoubleValue;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
  private static final double MAX_TRANSLATIONAL_POWER = 0.30;
  private static final double MAX_ROTATIONAL_POWER = 0.5;

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

    xController.setTolerance(Constants.VisionConstants.POSE_ALIGNMENT_TOLERANCE_XY);
    rController.setTolerance(Constants.VisionConstants.POSE_ALIGNMENT_TOLERANCE_R);

    rController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentX = laserCAN.getAverageDistance();
    double currentR = laserCAN.getAngleToWall();

    xErrorLog.append(xTarget - currentX);
    rErrorLog.append(rTarget - currentR);

    double xSpeed = 0;
    double rSpeed = 0;
    if (laserCAN.hasValidMeasurement()) {
      xSpeed =
          MathUtil.clamp(
              xController.calculate(currentX), -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);
      rSpeed =
          MathUtil.clamp(
              rController.calculate(currentR), -MAX_ROTATIONAL_POWER, MAX_ROTATIONAL_POWER);
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
    return xController.atSetpoint() && rController.atSetpoint();
  }
}
