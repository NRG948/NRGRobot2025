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
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef2 extends Command {
  private SwerveSubsystem drivetrain;
  private AprilTagSubsystem vision;

  private RobotPreferences.DoubleValue Px =
      new RobotPreferences.DoubleValue("AprilTag", "Reef P Constant for X", 0.5);
  private RobotPreferences.DoubleValue Py =
      new RobotPreferences.DoubleValue("AprilTag", "Reef P Constant for Y", 0.5);
  private RobotPreferences.DoubleValue Pr =
      new RobotPreferences.DoubleValue("AprilTag", "Reef P Constant for yaw", 0.5);

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

  /** Creates a new AlignToReef. */
  public AlignToReef2(Subsystems subsystems, ReefBranch targetBranch) {
    this.targetReefBranch = targetBranch;
    drivetrain = subsystems.drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
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
    int nearestTagId = findNearestReefTagID(currentRobotPose);
    targetPose =
        Constants.VisionConstants.REEF_SCORING_POSES.get(
            new Pair<Integer, ReefBranch>(nearestTagId, targetReefBranch));

    xController.setSetpoint(targetPose.getX());
    yController.setSetpoint(targetPose.getY());
    rController.setSetpoint(targetPose.getRotation().getDegrees());
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

    double xSpeed = MathUtil.clamp(xController.calculate(currentX), -0.25, 0.25);
    double ySpeed = MathUtil.clamp(yController.calculate(currentY), -0.25, 0.25);
    double rSpeed = MathUtil.clamp(rController.calculate(currentR), -0.25, 0.25);

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

  // TODO: get tag based on vision
  public static int findNearestReefTagID(Pose2d robotPose) {
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    var alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    int startingId = alliance.equals(Alliance.Red) ? 6 : 17;
    int nearestId = -1;
    double minDist = Double.MAX_VALUE;
    for (int id = startingId; id < startingId + 6; id++) {
      Transform2d tagToRobot = layout.getTagPose(id).get().toPose2d().minus(robotPose);
      double distSquared = Math.pow(tagToRobot.getX(), 2) + Math.pow(tagToRobot.getY(), 2);
      if (distSquared < minDist) {
        minDist = distSquared;
        nearestId = id;
      }
    }

    return nearestId;
  }
}
