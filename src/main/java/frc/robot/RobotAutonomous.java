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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;
import frc.robot.util.FieldUtils;
import java.util.function.DoubleSupplier;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.ReplanningConfig;

/**
 * This class creates and manages the user interface operators used to select and configure
 * autonomous routines.
 */
public class RobotAutonomous {
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Integer> delayChooser = new SendableChooser<>();
  private final RobotConfig config = Swerve.PARAMETERS.getPathplannerConfig();

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

    this.autoChooser = Autonomous.getChooser(subsystems, "frc.robot");
    autoChooser.onChange(Autos::preloadAuto);
    Autos.preloadAuto(autoChooser.getSelected());

    this.delayChooser.setDefaultOption("No Delay", (Integer) 0);
    for (var i = 1; i < 8; i++) {
      this.delayChooser.addOption(String.format("%d Second Delay", i), (Integer) i);
    }
  }

  /**
   * Returns the autonomous command selected in the chooser.
   *
   * @return The autonomous command selected in the chooser.
   */
  public Command getAutonomousCommand(Subsystems subsystems) {
    return Commands.sequence(
        Commands.waitSeconds(this.delayChooser.getSelected()), this.autoChooser.getSelected());
  }
}
