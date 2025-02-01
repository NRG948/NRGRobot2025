/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import com.nrg948.autonomous.Autonomous;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FieldUtils;
import java.util.function.DoubleSupplier;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.ReplanningConfig;

/**
 * This class creates and manages the user interface operators used to select and configure
 * autonomous routines.
 */
public class RobotAutonomous {
  private final SendableChooser<Command> chooser;

  private RobotConfig config = SwerveSubsystem.PARAMETERS.getValue().getPathplannerConfig();

  public RobotAutonomous(Subsystems subsystems, DoubleSupplier rotationFeedbackOverride) {
    AutoBuilder.configure(
        subsystems.drivetrain::getPosition,
        subsystems.drivetrain::resetPosition,
        subsystems.drivetrain::getChassisSpeeds,
        subsystems.drivetrain::setChassisSpeeds,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        config,
        FieldUtils::isRedAlliance,
        subsystems.drivetrain);

    PPHolonomicDriveController.overrideRotationFeedback(rotationFeedbackOverride);

    this.chooser = Autonomous.getChooser(subsystems, "frc.robot");
  }

  /**
   * Returns the autonomous command selected in the chooser.
   *
   * @return The autonomous command selected in the chooser.
   */
  public Command getAutonomousCommand(Subsystems subsystems) {

    return this.chooser.getSelected();
  }

  /**
   * Adds the autonomous layout to the shuffleboard tab.
   *
   * @param tab The tab to add the layout.
   */
  public void addShuffleboardLayout(ShuffleboardTab tab) {
    ShuffleboardLayout layout =
        tab.getLayout("Autonomous", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 2);
    layout.add("Routine", chooser);
  }
}
