/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static com.nrg948.preferences.RobotPreferences.DoubleValue;

import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;

/**
 * A {@link Command} that autonomous drives and aligns the robot to the specified branch of the
 * nearest reef side.
 */
@RobotPreferencesLayout(groupName = "AlignToReef", row = 0, column = 4, width = 2, height = 3)
public class AlignToReef extends Command {
  private static final DataLog LOG = DataLogManager.getLog();
  private static final double MAX_TRANSLATIONAL_POWER = 0.30;
  private static final double MAX_ROTATIONAL_POWER = 0.5;

  /** An enum that represents the two branches of the reef as viewed face on. */
  public enum ReefBranch {
    LEFT,
    RIGHT
  }

  @RobotPreferencesValue public static DoubleValue Px = new DoubleValue("AlignToReef", "X KP", 1);
  @RobotPreferencesValue public static DoubleValue Py = new DoubleValue("AlignToReef", "Y KP", 1);

  @RobotPreferencesValue
  public static DoubleValue Pr = new DoubleValue("AlignToReef", "R KP", 0.02);

  private final Swerve drivetrain;
  private final ReefBranch targetReefBranch;

  private final PIDController xController = new PIDController(Px.getValue(), 0, 0);
  private final PIDController yController = new PIDController(Py.getValue(), 0, 0);
  private final PIDController rController = new PIDController(Pr.getValue(), 0, 0);

  private final DoubleLogEntry xErrorLog = new DoubleLogEntry(LOG, "Vision/Reef X Error");
  private final DoubleLogEntry yErrorLog = new DoubleLogEntry(LOG, "Vision/Reef Y Error");
  private final DoubleLogEntry rErrorLog = new DoubleLogEntry(LOG, "Vision/Reef Yaw Error");

  private final DoubleLogEntry xTargetLog = new DoubleLogEntry(LOG, "Vision/Reef Target X");
  private final DoubleLogEntry yTargetLog = new DoubleLogEntry(LOG, "Vision/Reef Target Y");
  private final DoubleLogEntry rTargetLog = new DoubleLogEntry(LOG, "Vision/Reef Target Yaw");

  private double xTarget;
  private double yTarget;
  private double rTarget; // in degrees

  /** Creates a new {@link AlignToReef} command. */
  public AlignToReef(Subsystems subsystems, ReefBranch targetBranch) {
    setName(String.format("AlignToReef(%s)", targetBranch.name()));
    this.drivetrain = subsystems.drivetrain;
    this.targetReefBranch = targetBranch;

    // Declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Figure out the robot's target pose & set the PID gains and setpoints.
    Pose2d currentRobotPose = drivetrain.getPosition();
    Pose2d nearestTagPose = currentRobotPose.nearest(FieldUtils.getReefAprilTags());
    double v, h, d;
    v = Constants.RobotConstants.ODOMETRY_CENTER_TO_FRONT_BUMPER_DELTA_X;
    h = Constants.RobotConstants.CORAL_OFFSET_Y;
    d = Constants.VisionConstants.BRANCH_TO_REEF_APRILTAG;

    var targetPose =
        nearestTagPose.plus(
            new Transform2d(
                v, (targetReefBranch.equals(ReefBranch.RIGHT) ? d : -d) - h, Rotation2d.k180deg));
    System.out.println("TARGET pose: " + targetPose);
    System.out.println("Target Branch: " + targetReefBranch);

    xTarget = targetPose.getX();
    yTarget = targetPose.getY();
    rTarget = targetPose.getRotation().getDegrees();

    xTargetLog.append(xTarget);
    yTargetLog.append(yTarget);
    rTargetLog.append(rTarget);

    xController.setSetpoint(xTarget);
    yController.setSetpoint(yTarget);
    rController.setSetpoint(rTarget);

    xController.setPID(Px.getValue(), 0, 0);
    yController.setPID(Py.getValue(), 0, 0);
    rController.setPID(Pr.getValue(), 0, 0);

    xController.setTolerance(Constants.VisionConstants.REEF_ALIGNMENT_TOLERANCE_XY);
    yController.setTolerance(Constants.VisionConstants.REEF_ALIGNMENT_TOLERANCE_XY);
    rController.setTolerance(Constants.VisionConstants.REEF_ALIGNMENT_TOLERANCE_R);

    rController.enableContinuousInput(-180, 180);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // run pid
    Pose2d currentRobotPose = drivetrain.getPosition();
    double currentX = currentRobotPose.getX();
    double currentY = currentRobotPose.getY();
    double currentR = currentRobotPose.getRotation().getDegrees();

    xErrorLog.append(xTarget - currentX);
    yErrorLog.append(yTarget - currentY);
    rErrorLog.append(rTarget - currentR);

    double xSpeed =
        MathUtil.clamp(
            xController.calculate(currentX), -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);
    double ySpeed =
        MathUtil.clamp(
            yController.calculate(currentY), -MAX_TRANSLATIONAL_POWER, MAX_TRANSLATIONAL_POWER);
    double rSpeed =
        MathUtil.clamp(
            rController.calculate(currentR), -MAX_ROTATIONAL_POWER, MAX_ROTATIONAL_POWER);

    drivetrain.drive(xSpeed, ySpeed, rSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && rController.atSetpoint();
  }
}
