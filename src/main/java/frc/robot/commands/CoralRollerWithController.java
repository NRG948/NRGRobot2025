/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CoralRoller;
import frc.robot.subsystems.Subsystems;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralRollerWithController extends Command {
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue DEADBAND =
      new RobotPreferences.DoubleValue("CoralRoller", "Deadband", 0.1);

  private final CoralRoller coralRoller;
  private final CommandXboxController xboxController;

  /** Creates a new CoralRollerWithController. */
  public CoralRollerWithController(Subsystems subsystems, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    coralRoller = subsystems.coralRoller;
    this.xboxController = xboxController;
    addRequirements(coralRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -xboxController.getLeftY();
    speed = MathUtil.applyDeadband(speed, DEADBAND.getValue());

    coralRoller.setGoalVelocity(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralRoller.setGoalVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
