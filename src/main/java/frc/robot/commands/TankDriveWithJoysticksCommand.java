// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystems;
import frc.robot.subsystems.TankDriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TankDriveWithJoysticksCommand extends Command {

private TankDriveSubsystem drivetrain;
private XboxController controller;

  /** Creates a new TankDriveWithJoysticksCommand. */
  public TankDriveWithJoysticksCommand(TankDriveSubsystem drivetrain, XboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double left = -controller.getLeftY();
    double right = -controller.getRightY();
    drivetrain.tankDrive(left, right);
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
