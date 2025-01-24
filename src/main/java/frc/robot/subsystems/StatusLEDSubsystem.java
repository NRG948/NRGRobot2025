// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ColorConstants.RED;
import static frc.robot.Constants.RobotConstants.LEDSegment.STATUS_FIRST_LED;
import static frc.robot.Constants.RobotConstants.LEDSegment.STATUS_LED_COUNT;

public class StatusLEDSubsystem extends LEDSubsystem {
  /** Creates a new StatusLEDSubsystem. */
  public StatusLEDSubsystem() {
    super(STATUS_FIRST_LED, STATUS_LED_COUNT);
    leds.fill(RED);
    leds.commitColor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
