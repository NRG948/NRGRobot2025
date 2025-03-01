/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToReef.ReefPosition;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefLateralAlignment extends Command {
  private static final double SCALE_FACTOR = 0.3;

  private Swerve drivetrain;
  private AprilTag vision;
  private int targetID;
  private ProfiledPIDController controller;
  private ReefPosition reefPosition;
  private TrapezoidProfile.Constraints constraints;

  /** Creates a new TagLateralAlignment. */
  public ReefLateralAlignment(Subsystems subsystems, ReefPosition reefPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystems.drivetrain;
    vision = subsystems.frontCamera.get();
    this.reefPosition = reefPosition;
    constraints =
        new TrapezoidProfile.Constraints(
            Swerve.getMaxSpeed() * SCALE_FACTOR, Swerve.getMaxAcceleration() * SCALE_FACTOR);
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

      case RIGHT_BRANCH:
        targetY = VisionConstants.BRANCH_TO_REEF_APRILTAG;

      default:
        targetY = 0;
        break;
    }
    controller.setGoal(targetY);
    controller.setTolerance(VisionConstants.POSE_ALIGNMENT_TOLERANCE_XY);
    controller.setPID(AlignToPose.Py.getValue(), 0, 0);
    controller.setConstraints(constraints);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> optionalTarget = vision.getTarget(targetID);
    if (optionalTarget.isEmpty()) {
      // ?
    }
    PhotonTrackedTarget target = optionalTarget.get();
    double yDist = target.getBestCameraToTarget().getY();
    double pidOutput = controller.calculate(yDist);
    drivetrain.drive(0, pidOutput, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atGoal();
  }
}
