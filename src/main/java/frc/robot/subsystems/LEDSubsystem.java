/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.Colors;
import frc.robot.util.LEDSegment;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  protected final LEDSegment leds;

  /**
   * Creates a new LEDSubsystem.
   *
   * @param firstLED The first LED index.
   * @param ledCount The number of LEDs in the segment.
   */
  public LEDSubsystem(int firstLED, int ledCount) {
    leds = new LEDSegment(firstLED, ledCount);
  }

  /**
   * Gets the number of LEDs in the segment.
   *
   * @return The number of LEDs in the segment.
   */
  public int getLEDCount() {
    return leds.getLEDCount();
  }

  /**
   * Sets the color of the LED at the specified index.
   *
   * @param color The color to set the LED to.
   * @param index The index of the LED to set.
   */
  public void setColor(Colors color, int index) {
    leds.setColor(color, index);
  }

  /**
   * Sets the color of the LED at the specified index.
   *
   * @param color The color to set the LED to.
   * @param index The index of the LED to set.
   */
  public void setColor(Color8Bit color, int index) {
    leds.setColor(color, index);
  }

  /**
   * Fills the segment with the specified color.
   *
   * @param color The color to fill the segment with.
   */
  public void fillColor(Colors color) {
    leds.fill(color.getColor());
  }

  /**
   * Fills the segment with the specified color.
   *
   * @param color The color to fill the segment with.
   */
  public void fillColor(Color8Bit color) {
    leds.fill(color);
  }

  /**
   * Fills the segment with the specified color and displays it to the LED segment.
   *
   * @param color The color to fill the segment with.
   */
  public void fillAndCommitColor(Colors color) {
    leds.fill(color.getColor());
    commitColor();
  }

  /**
   * Fills the segment with the specified color and displays it to the LED segment.
   *
   * @param color The color to fill the segment with.
   */
  public void fillAndCommitColor(Color8Bit color) {
    leds.fill(color);
    commitColor();
  }

  /** Displays the current color data on the LED strip. */
  public void commitColor() {
    leds.commitColor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
