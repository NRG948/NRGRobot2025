/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** An enum representing common RGB colors used with the status LEDs. */
public enum Colors {
  BLACK(0, 0, 0),
  WHITE(200, 200, 200),
  RED(255, 0, 0),
  ORANGE(255, 119, 0),
  YELLOW(255, 165, 0),
  GREEN(0, 204, 0),
  BLUE(0, 0, 204),
  PURPLE(238, 80, 255),
  PINK(255, 5, 100),
  LIGHT_BLUE(56, 197, 252);

  private final Color8Bit color;

  /** Constructs a variant of this enum. */
  Colors(int red, int green, int blue) {
    color = new Color8Bit(red, green, blue);
  }

  /** Returns the red component of this enum variant. */
  public int getRed() {
    return color.red;
  }

  /** Returns the green component of this enum variant. */
  public int getGreen() {
    return color.green;
  }

  /** Returns the blue component of this enum variant. */
  public int getBlue() {
    return color.blue;
  }

  /** Returns the color of this enum variant. */
  public Color8Bit getColor() {
    return color;
  }
}
