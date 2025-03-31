/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferences.DoubleValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.RateLimiter;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveUsingController extends Command {
  private static final double RUMBLE_MIN_G = 1.0;
  private static final double RUMBLE_MAX_G = 8.0;
  public static final RobotPreferences.DoubleValue minPower =
      new DoubleValue("Drive", "Min Power", 0.3);
  public static final RobotPreferences.DoubleValue climbAccelLimit =
      new DoubleValue("Drive", "Climb Accel Limit", 0.3);
  private static final double DEADBAND = 0.08;

  private final Swerve drivetrain;
  private final Elevator elevator;
  private final CommandXboxController xboxController;
  private final RateLimiter xAccelLimiter = new RateLimiter(Double.MAX_VALUE);
  private final RateLimiter yAccelLimiter = new RateLimiter(Double.MAX_VALUE);

  /** Creates a new DriveUsingController. */
  public DriveUsingController(Subsystems subsystems, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystems.drivetrain;
    elevator = subsystems.elevator;
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
    double inputScalar = Math.max(1.0 - xboxController.getRightTriggerAxis(), minPower.getValue());

    // Applies deadbands to x and y joystick values and multiples all
    // values with inputScalar which allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;
    rSpeed = MathUtil.applyDeadband(rSpeed, DEADBAND) * inputScalar;

    // Limit robot translation accleration based on elevator height or pulling left trigger.
    var accelLimit =
        xboxController.leftTrigger().getAsBoolean()
            ? climbAccelLimit.getValue()
            : elevator.getCurrentElevatorLevel().getAccelLimit();
    xAccelLimiter.setRateLimit(accelLimit);
    yAccelLimiter.setRateLimit(accelLimit);
    xSpeed = xAccelLimiter.calculate(xSpeed);
    ySpeed = xAccelLimiter.calculate(ySpeed);

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
