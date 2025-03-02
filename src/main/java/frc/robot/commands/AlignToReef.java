/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.util.FieldUtils.getRobotPoseForNearestReefAprilTag;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.ReefPosition;

/**
 * A {@link Command} that autonomous drives and aligns the robot to the specified position of the
 * nearest reef side.
 */
public class AlignToReef extends AlignToPose {

  private final ReefPosition reefPosition;

  /** Creates a new {@link AlignToReef} command. */
  public AlignToReef(Subsystems subsystems, ReefPosition reefPosition) {
    super(subsystems);
    setName(String.format("AlignToReef(%s)", reefPosition.name()));
    this.reefPosition = reefPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Figure out the robot's target pose.
    targetPose = getRobotPoseForNearestReefAprilTag(drivetrain.getPosition(), reefPosition);

    super.initialize();
  }
}
