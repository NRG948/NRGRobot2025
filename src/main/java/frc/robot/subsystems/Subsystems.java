/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class Subsystems {
  public final Elevator elevator = new Elevator();

  public final Subsystem[] all = new Subsystem[] {elevator};
}
