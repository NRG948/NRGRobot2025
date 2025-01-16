// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  private static final double X_GOAL = 0.15; // for bumper
  private static final double Y_OFFSET = 0.165;

  private SwerveSubsystem drivetrain;
  private AprilTagSubsystem vision;

  private RobotPreferences.DoubleValue Px = new RobotPreferences.DoubleValue("AprilTag", "P Constant for X", 0.5);
  private RobotPreferences.DoubleValue Py = new RobotPreferences.DoubleValue("AprilTag", "P Constant for Y", 0.5);
  private RobotPreferences.DoubleValue Pr = new RobotPreferences.DoubleValue("AprilTag", "P Constant for yaw", 0.5);

  private PIDController xController = new PIDController(Px.getValue(), 0, 0);
  private PIDController yController = new PIDController(Py.getValue(), 0, 0);
  private PIDController rController = new PIDController(Pr.getValue(), 0, 0);

  private DoubleLogEntry xErrorLog = new DoubleLogEntry(DataLogManager.getLog(), "Vision/Reef X Error");
  private DoubleLogEntry yErrorLog = new DoubleLogEntry(DataLogManager.getLog(), "Vision/Reef Y Error");
  private DoubleLogEntry rErrorLog = new DoubleLogEntry(DataLogManager.getLog(), "Vision/Reef Yaw Error");

  private int targetId;

  /*
  idea for improvement:
    - use odometry at the start
    - PID one at the time instead of all three
    - better ending condition
  */ 
  
  /** Creates a new AlignToReef. */
  public AlignToReef(Subsystems subsystems) {
    targetId = 18; // placeholder, need to discuss with driveteam how they want to select reef
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(X_GOAL);
    yController.setSetpoint(Y_OFFSET); // TODO: add branch selection by adding negative sign
    rController.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Transform3d tagToCamera = vision.getTranformToTag(targetId); // placeholder ID
    double dx = tagToCamera.getX();
    double dy = tagToCamera.getY();
    double dr = vision.getAngleToTarget(targetId);

    xErrorLog.append(dx);
    yErrorLog.append(dy);
    rErrorLog.append(dr);

    drivetrain.drive(
      MathUtil.clamp(xController.calculate(dx), -0.25, 0.25),
      MathUtil.clamp(yController.calculate(dy), -0.25, 0.25),
      MathUtil.clamp(rController.calculate(dr), -0.25, 0.25),
      false);
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
}
