// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Swerve;



/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WheelCharacterization extends Command {
  
  Subsystem subsystem;
  Swerve swerveSubsystem;
 
  private final HolonomicDriveController controller;
  private final double DISTANCE; // Goal distance in meters
  private Translation2d translation;
  private double maxSpeed;
  private TrapezoidProfile profile;
  private TrapezoidProfile.State initialState;
  private TrapezoidProfile.State goalState;
  private Rotation2d heading;
  private Pose2d initialPose;
  private Rotation2d orientation;
  private Timer timer = new Timer();

/** Creates a new WheelDiameterCalibration. */
  public WheelCharacterization(Swerve driveTrain, double maxSpeed, Translation2d translation) {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(subsystem);
  
    this.swerveSubsystem = driveTrain;
    this.translation = translation;
    this.maxSpeed = maxSpeed;
    this.DISTANCE = 3.0;
    this.controller = driveTrain.createDriveController();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxSpeed, swerveSubsystem.getMaxAcceleration()));
    initialState = new TrapezoidProfile.State(0,0);
    goalState = new TrapezoidProfile.State(DISTANCE, 0);
    heading = translation.getAngle();
    initialPose = swerveSubsystem.getPosition();
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    TrapezoidProfile.State state = profile.calculate(timer.get(), initialState, goalState); 
    Pose2d nextPose = new Pose2d(initialPose.getTranslation(), heading);
    ChassisSpeeds speeds = controller.calculate(swerveSubsystem.getPosition(), nextPose, state.velocity, orientation);
    swerveSubsystem.setChassisSpeeds(speeds);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    swerveSubsystem.stopMotors();
    double expectedDistance = swerveSubsystem.getPosition().getY();

    double actualDiameter = RobotConstants.WHEEL_DIAMETER * DISTANCE / expectedDistance;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
