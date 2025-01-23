/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.Common;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AlignToReef.ReefBranch;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    Common.init("frc.robot");

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // calculate the scoring positions for each reef branch
    double v, h, d;
    v = Constants.RobotConstants.ROBOT_LENGTH / 2;
    h = Constants.RobotConstants.CORAL_OFFSET_Y;
    d = Constants.VisionConstants.BRANCH_TO_REEF_APRILTAG;

    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    // 6-11 Red Reef Apriltags, 17-22 Blue Reef Apriltags
    for (int i = 0; i < 6; i++) {

      // blue
      Pose2d tagPoseBlue = fieldLayout.getTagPose(i + 17).get().toPose2d();
      Pose2d leftTargetPoseBlue = tagPoseBlue.plus(new Transform2d(v, -d - h, Rotation2d.k180deg));
      Pose2d rightTargetPoseBlue = tagPoseBlue.plus(new Transform2d(v, d - h, Rotation2d.k180deg));

      Constants.VisionConstants.REEF_SCORING_POSES.put(
          new Pair<Integer, ReefBranch>(i + 17, ReefBranch.LEFT), leftTargetPoseBlue);
      Constants.VisionConstants.REEF_SCORING_POSES.put(
          new Pair<Integer, ReefBranch>(i + 17, ReefBranch.RIGHT), rightTargetPoseBlue);

      // red
      Pose2d tagPoseRed = fieldLayout.getTagPose(i + 6).get().toPose2d();
      Pose2d leftTargetPoseRed = tagPoseRed.plus(new Transform2d(v, -d - h, Rotation2d.k180deg));
      Pose2d rightTargetPoseRed = tagPoseRed.plus(new Transform2d(v, d - h, Rotation2d.k180deg));

      Constants.VisionConstants.REEF_SCORING_POSES.put(
          new Pair<Integer, ReefBranch>(i + 6, ReefBranch.LEFT), leftTargetPoseRed);
      Constants.VisionConstants.REEF_SCORING_POSES.put(
          new Pair<Integer, ReefBranch>(i + 6, ReefBranch.RIGHT), rightTargetPoseRed);
    }

    /**
     * Tag + Branch: Pair(21, RIGHT), TargetPose: Pose2d(Translation2d(X: 5.69, Y: 3.99),
     * Rotation2d(Rads: 3.14, Deg: 180.00)) Tag + Branch: Pair(10, LEFT), TargetPose:
     * Pose2d(Translation2d(X: 11.86, Y: 4.39), Rotation2d(Rads: -0.00, Deg: -0.00)) Tag + Branch:
     * Pair(20, RIGHT), TargetPose: Pose2d(Translation2d(X: 5.12, Y: 5.05), Rotation2d(Rads: -2.09,
     * Deg: -120.00)) Tag + Branch: Pair(11, LEFT), TargetPose: Pose2d(Translation2d(X: 12.14, Y:
     * 3.17), Rotation2d(Rads: 1.05, Deg: 60.00)) Tag + Branch: Pair(22, RIGHT), TargetPose:
     * Pose2d(Translation2d(X: 5.06, Y: 2.97), Rotation2d(Rads: 2.09, Deg: 120.00)) Tag + Branch:
     * Pair(17, LEFT), TargetPose: Pose2d(Translation2d(X: 3.57, Y: 3.17), Rotation2d(Rads: 1.05,
     * Deg: 60.00)) Tag + Branch: Pair(18, LEFT), TargetPose: Pose2d(Translation2d(X: 3.29, Y:
     * 4.39), Rotation2d(Rads: -0.00, Deg: -0.00)) Tag + Branch: Pair(19, LEFT), TargetPose:
     * Pose2d(Translation2d(X: 4.21, Y: 5.25), Rotation2d(Rads: -1.05, Deg: -60.00)) Tag + Branch:
     * Pair(20, LEFT), TargetPose: Pose2d(Translation2d(X: 5.41, Y: 4.88), Rotation2d(Rads: -2.09,
     * Deg: -120.00)) Tag + Branch: Pair(21, LEFT), TargetPose: Pose2d(Translation2d(X: 5.69, Y:
     * 3.66), Rotation2d(Rads: 3.14, Deg: 180.00)) Tag + Branch: Pair(17, RIGHT), TargetPose:
     * Pose2d(Translation2d(X: 3.86, Y: 3.00), Rotation2d(Rads: 1.05, Deg: 60.00)) Tag + Branch:
     * Pair(22, LEFT), TargetPose: Pose2d(Translation2d(X: 4.77, Y: 2.80), Rotation2d(Rads: 2.09,
     * Deg: 120.00)) Tag + Branch: Pair(19, RIGHT), TargetPose: Pose2d(Translation2d(X: 3.92, Y:
     * 5.08), Rotation2d(Rads: -1.05, Deg: -60.00)) Tag + Branch: Pair(18, RIGHT), TargetPose:
     * Pose2d(Translation2d(X: 3.29, Y: 4.06), Rotation2d(Rads: -0.00, Deg: -0.00)) Tag + Branch:
     * Pair(7, RIGHT), TargetPose: Pose2d(Translation2d(X: 14.26, Y: 3.99), Rotation2d(Rads: 3.14,
     * Deg: 180.00)) Tag + Branch: Pair(6, RIGHT), TargetPose: Pose2d(Translation2d(X: 13.63, Y:
     * 2.97), Rotation2d(Rads: 2.09, Deg: 120.00)) Tag + Branch: Pair(9, RIGHT), TargetPose:
     * Pose2d(Translation2d(X: 12.49, Y: 5.08), Rotation2d(Rads: -1.05, Deg: -60.00)) Tag + Branch:
     * Pair(8, RIGHT), TargetPose: Pose2d(Translation2d(X: 13.69, Y: 5.05), Rotation2d(Rads: -2.09,
     * Deg: -120.00)) Tag + Branch: Pair(11, RIGHT), TargetPose: Pose2d(Translation2d(X: 12.43, Y:
     * 3.00), Rotation2d(Rads: 1.05, Deg: 60.00)) Tag + Branch: Pair(10, RIGHT), TargetPose:
     * Pose2d(Translation2d(X: 11.86, Y: 4.06), Rotation2d(Rads: -0.00, Deg: -0.00)) Tag + Branch:
     * Pair(6, LEFT), TargetPose: Pose2d(Translation2d(X: 13.34, Y: 2.80), Rotation2d(Rads: 2.09,
     * Deg: 120.00)) Tag + Branch: Pair(7, LEFT), TargetPose: Pose2d(Translation2d(X: 14.26, Y:
     * 3.66), Rotation2d(Rads: 3.14, Deg: 180.00)) Tag + Branch: Pair(8, LEFT), TargetPose:
     * Pose2d(Translation2d(X: 13.98, Y: 4.88), Rotation2d(Rads: -2.09, Deg: -120.00)) Tag + Branch:
     * Pair(9, LEFT), TargetPose: Pose2d(Translation2d(X: 12.77, Y: 5.25), Rotation2d(Rads: -1.05,
     * Deg: -60.00))
     */
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    m_robotContainer.periodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
