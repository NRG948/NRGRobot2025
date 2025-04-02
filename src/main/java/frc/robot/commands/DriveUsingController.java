/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveUsingController extends Command {

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue RIGHT_TRIGGER_SCALAR =
      new RobotPreferences.DoubleValue("Drive", "Right Trigger Scalar", 0.25);

  private static final double RUMBLE_MIN_G = 1.0;
  private static final double RUMBLE_MAX_G = 8.0;

  private static final double DEADBAND = 0.08;

  private final Swerve drivetrain;
  private final CommandXboxController xboxController;

  /** Creates a command which allows a human to drive the robot using an Xbox controller. */
  public DriveUsingController(Subsystems subsystems, CommandXboxController xboxController) {
    drivetrain = subsystems.drivetrain;
    this.xboxController = xboxController;

    // Use addRequirements() here to declare subsystem dependencies.
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

    // The `powerScalar` linearly scales the robot's drive power from 1.0 (when the right trigger is
    // not pressed) down to RIGHT_TRIGGER_SCALAR (when the right trigger is fully depressed).
    double powerScalar =
        (RIGHT_TRIGGER_SCALAR.getValue() - 1.0) * xboxController.getRightTriggerAxis() + 1.0;

    // Applies deadbands to the x, y, and rotation joystick values and then multiplies all speeds by
    // the powerScalar, which allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * powerScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * powerScalar;
    rSpeed = MathUtil.applyDeadband(rSpeed, DEADBAND) * powerScalar;

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
