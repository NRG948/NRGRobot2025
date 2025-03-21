/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences.DoubleValue;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveUntilHitObstacle extends Command {
  @RobotPreferencesValue
  public static DoubleValue SPEED_THRESHOLD =
      new DoubleValue("AlignToPose", "Stopping Speed Threshold", 0.1);

  private Swerve drivetrain;
  private double drivePower;

  /** Creates a new DriveUntilHitObstacle. */
  public DriveUntilHitObstacle(Subsystems subsystems, double drivePower) {
    drivetrain = subsystems.drivetrain;
    this.drivePower = drivePower;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(drivePower, 0, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    ChassisSpeeds chassisSpeeds = drivetrain.getChassisSpeeds();
    return Math.pow(chassisSpeeds.vxMetersPerSecond, 2)
            + Math.pow(chassisSpeeds.vyMetersPerSecond, 2)
        < Math.pow(SPEED_THRESHOLD.getValue(), 2);
  }
}
