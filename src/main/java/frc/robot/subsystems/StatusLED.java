/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.LEDSegment.STATUS_FIRST_LED;
import static frc.robot.Constants.RobotConstants.LEDSegment.STATUS_LED_COUNT;
import static frc.robot.parameters.Colors.RED;

/** A subsystem to control the status LEDs. */
public final class StatusLED extends LEDSubsystem {
  /** Creates a new StatusLEDSubsystem. */
  public StatusLED() {
    super(STATUS_FIRST_LED, STATUS_LED_COUNT);
    leds.fill(RED);
    leds.commitColor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
