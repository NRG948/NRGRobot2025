/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveUsingController extends Command {
  private static final double RUMBLE_MIN_G = 1.0;
  private static final double RUMBLE_MAX_G = 8.0;

  private static final double DEADBAND = 0.08;

  private final Swerve drivetrain;
  private final CommandXboxController xboxController;

  /** Creates a new DriveUsingController. */
  public DriveUsingController(Subsystems subsystems, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystems.drivetrain;
    this.xboxController = xboxController;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rSpeed = -xboxController.getRightX();
    double xSpeed = -xboxController.getLeftY();
    double ySpeed = -xboxController.getLeftX();
    double inputScalar = Math.max(1.0 - xboxController.getRightTriggerAxis(), 0.15);

    // Applies deadbands to x and y joystick values and multiples all
    // values with inputScalar which allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;
    rSpeed = MathUtil.applyDeadband(rSpeed, DEADBAND) * inputScalar;

    drivetrain.drive(xSpeed, ySpeed, rSpeed, true);

    if (Swerve.ENABLE_RUMBLE.getValue()) {
      // Rumbles the driver controller based on a exponential scale based on acceleration between
      // min and max.
      double rumblePower =
          MathUtil.inverseInterpolate(RUMBLE_MIN_G, RUMBLE_MAX_G, drivetrain.getAcceleration());
      xboxController.setRumble(RumbleType.kBothRumble, rumblePower * rumblePower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
