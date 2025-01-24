/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LEDSegment;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  protected final LEDSegment leds;

  // TODO: update constructor
  public LEDSubsystem(int firstLED, int ledCount) {
    leds = new LEDSegment(firstLED, ledCount);
  }

  public void setColor(Color8Bit color, int index) {
    leds.setColor(color, index);
  }

  public void fillAndCommitColor(Color8Bit color) {
    leds.fill(color);
    commitColor();
  }

  public void commitColor() {
    leds.commitColor();
  }

  public void fillColor(Color8Bit color) {
    leds.fill(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
