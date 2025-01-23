/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command { // TODO:
  private static final double MAX_TRANSLATIONAL_POWER = 0.30;
  private static final double MAX_ROTATIONAL_POWER = 0.5;

  private SwerveSubsystem drivetrain;

  // they don't seem to appear. TODO: make preferences appear in shuffleboard
  private RobotPreferences.DoubleValue Px =
      new RobotPreferences.DoubleValue("AprilTag", "Reef P Constant for X", 0.5);
  private RobotPreferences.DoubleValue Py =
      new RobotPreferences.DoubleValue("AprilTag", "Reef P Constant for Y", 0.5);
  private RobotPreferences.DoubleValue Pr =
      new RobotPreferences.DoubleValue("AprilTag", "Reef P Constant for yaw", 0.05);

  private PIDController xController = new PIDController(Px.getValue(), 0, 0);
  private PIDController yController = new PIDController(Py.getValue(), 0, 0);
  private PIDController rController = new PIDController(Pr.getValue(), 0, 0);

  private DoubleLogEntry xErrorLog =
      new DoubleLogEntry(DataLogManager.getLog(), "Vision/Reef X Error");
  private DoubleLogEntry yErrorLog =
      new DoubleLogEntry(DataLogManager.getLog(), "Vision/Reef Y Error");
  private DoubleLogEntry rErrorLog =
      new DoubleLogEntry(DataLogManager.getLog(), "Vision/Reef Yaw Error");

  private ReefBranch targetReefBranch;
  private Pose2d targetPose;

  private static final ArrayList<Pose2d> redReefTags = new ArrayList<>(6);
  private static final ArrayList<Pose2d> blueReefTags = new ArrayList<>(6);

  /** Creates a new AlignToReef. */
  public AlignToReef(Subsystems subsystems, ReefBranch targetBranch) {
    this.targetReefBranch = targetBranch;
    drivetrain = subsystems.drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    // 6-11 Red Reef Apriltags, 17-22 Blue Reef Apriltags
    for (int i = 0; i < 6; i++) {
      redReefTags.add(layout.getTagPose(i + 6).get().toPose2d());
      blueReefTags.add(layout.getTagPose(i + 17).get().toPose2d());
    }
  }

  /** An enum that represents the two branches of the reef as viewed face on. */
  public enum ReefBranch {
    LEFT,
    RIGHT
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // figure out target pose & set setpoints
    Pose2d currentRobotPose = drivetrain.getPosition();
    Pose2d nearestTagPose = findNearestReefTag(currentRobotPose);
    double v, h, d;
    v = Constants.RobotConstants.ROBOT_LENGTH / 2;
    h = Constants.RobotConstants.CORAL_OFFSET_Y;
    d = Constants.VisionConstants.BRANCH_TO_REEF_APRILTAG;

    targetPose =
        nearestTagPose.plus(
            new Transform2d(
                v, (targetReefBranch.equals(ReefBranch.RIGHT) ? d : -d) - h, Rotation2d.k180deg));
    System.out.println("TARGET pose: " + targetPose);
    System.out.println("Target Branch: " + targetReefBranch);

    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    rController.setSetpoint(targetPose.getRotation().getDegrees());

    xController.setTolerance(Constants.VisionConstants.REEF_ALIGNMENT_TOLERANCE_XY);
    yController.setTolerance(Constants.VisionConstants.REEF_ALIGNMENT_TOLERANCE_XY);
    rController.setTolerance(Constants.VisionConstants.REEF_ALIGNMENT_TOLERANCE_R);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // run pid
    Pose2d currentRobotPose = drivetrain.getPosition();
    double currentX = currentRobotPose.getX();
    double currentY = currentRobotPose.getY();
    double currentR = currentRobotPose.getRotation().getDegrees();

    xErrorLog.append(targetPose.getX() - currentX);
    yErrorLog.append(targetPose.getY() - currentY);
    rErrorLog.append(targetPose.getRotation().getDegrees() - currentR);

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

  public static Pose2d findNearestReefTag(Pose2d robotPose) {
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return robotPose.nearest(alliance.equals(Alliance.Red) ? redReefTags : blueReefTags);
  }
}
