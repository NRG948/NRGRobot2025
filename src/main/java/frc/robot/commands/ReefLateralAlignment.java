/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences.DoubleValue;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;
import frc.robot.util.ReefPosition;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * A command that aligns the robot to the specified reef position by first rotating the robot to
 * face the nearest reef side and then moving laterally to a position directly in front of the
 * position.
 */
public class ReefLateralAlignment extends Command {
  private static final double SCALE_FACTOR = 0.3;

  @RobotPreferencesValue
  public static DoubleValue Pr = new DoubleValue("AlignToPose", "Lateral R KP", 5.0);

  @RobotPreferencesValue
  public static DoubleValue Py = new DoubleValue("AlignToPose", "Lateral Y KP", 3.0);

  private Swerve drivetrain;
  private AprilTag vision;
  private int targetID;
  private ProfiledPIDController yController;
  private ProfiledPIDController rController;
  private ReefPosition reefPosition;
  private boolean isAligningY = false;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();

  /** Creates a new TagLateralAlignment. */
  public ReefLateralAlignment(Subsystems subsystems, ReefPosition reefPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystems.drivetrain;
    vision = subsystems.frontCamera.get();
    this.reefPosition = reefPosition;

    yController =
        new ProfiledPIDController(
            Py.getValue(),
            0,
            0,
            new TrapezoidProfile.Constraints(
                Swerve.getMaxSpeed() * SCALE_FACTOR, Swerve.getMaxAcceleration() * SCALE_FACTOR));
    rController =
        new ProfiledPIDController(
            Math.toRadians(Pr.getValue()), 0, 0, Swerve.getRotationalConstraints());
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetID = vision.getBestTarget().getFiducialId(); // bad, but good enough for now?
    double targetY;
    switch (reefPosition) {
      case LEFT_BRANCH:
        targetY = -VisionConstants.BRANCH_TO_REEF_APRILTAG;
        break;

      case CENTER_REEF:
        targetY = 0;
        break;

      case RIGHT_BRANCH:
        targetY = VisionConstants.BRANCH_TO_REEF_APRILTAG;
        break;

      default:
        targetY = 0;
        break;
    }

    yController.setGoal(targetY);
    yController.setTolerance(VisionConstants.POSE_ALIGNMENT_TOLERANCE_XY);
    yController.setPID(Py.getValue(), 0, 0);

    Pose2d currentRobotPose = drivetrain.getPosition();
    Pose2d nearestTagPose = currentRobotPose.nearest(FieldUtils.getReefAprilTags());
    rController.setGoal(nearestTagPose.getRotation().rotateBy(Rotation2d.k180deg).getRadians());
    rController.setTolerance(Math.toRadians(VisionConstants.POSE_ALIGNMENT_TOLERANCE_R));
    rController.setPID(Pr.getValue(), 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rController.atGoal()) {
      isAligningY = true;
    }

    Optional<PhotonTrackedTarget> optionalTarget = vision.getTarget(targetID);
    if (optionalTarget.isEmpty()) {
      return;
    }

    double rOutput = rController.calculate(drivetrain.getPosition().getRotation().getRadians());
    chassisSpeeds.omegaRadiansPerSecond = rOutput;

    if (isAligningY) {
      PhotonTrackedTarget target = optionalTarget.get();
      double yDist = target.getBestCameraToTarget().getY();
      double yOutput = -yController.calculate(yDist);
      chassisSpeeds.vyMetersPerSecond = yOutput;
    }

    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    chassisSpeeds.vyMetersPerSecond = 0;
    drivetrain.setChassisSpeeds(chassisSpeeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return yController.atGoal();
  }
}
