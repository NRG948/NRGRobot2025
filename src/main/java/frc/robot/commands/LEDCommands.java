/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.parameters.Colors.GREEN;
import static frc.robot.parameters.Colors.RED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.Colors;
import frc.robot.subsystems.StatusLED;
import frc.robot.subsystems.Subsystems;

/** A namespace for LED command factory methods. */
public final class LEDCommands {
  private static final double BLINK_DURATION = 1.0;

  /**
   * Returns a command that sets the color of the status LEDs.
   *
   * @param statusLED The status LED subsystem.
   * @param color The color to set.
   * @return A command that sets the color of the status LED.
   */
  public static Command setColor(StatusLED statusLED, Colors color) {
    return Commands.runOnce(() -> statusLED.fillAndCommitColor(color), statusLED)
        .withName(String.format("SetColor(%s)", color.name()));
  }

  /**
   * Returns a command that blinks the status LEDs green for one second and then sets the color to
   * solid green when a coral is acquired.
   *
   * @param statusLED The status LED subsystem.
   * @param period The period of the blink.
   * @return A command that blinks the status LEDs.
   */
  public static Command indicateCoralAcquired(Subsystems subsystem) {
    return Commands.sequence(
            new BlinkColor(subsystem.statusLEDs, GREEN, BLINK_DURATION),
            setColor(subsystem.statusLEDs, GREEN),
            Commands.idle(subsystem.statusLEDs).until(() -> !subsystem.coralRoller.hasCoral()))
        .withName("IndicateCoralAcquired");
  }

  /**
   * Returns a command that blinks the status LEDs green for one second and then sets the color to
   * solid green when an algae is acquired.
   *
   * @param statusLED The status LED subsystem.
   * @param period The period of the blink.
   * @return A command that blinks the status LEDs.
   */
  public static Command indicateAlgaeAcquired(Subsystems subsystem) {
    return Commands.sequence(
            new BlinkColor(subsystem.statusLEDs, GREEN, BLINK_DURATION),
            setColor(subsystem.statusLEDs, GREEN),
            Commands.idle(subsystem.statusLEDs).until(() -> !subsystem.algaeGrabber.hasAlgae()))
        .withName("IndicateAlgaeAcquired");
  }

  /**
   * Returns a command that blinks the status LEDs red while a condition is true.
   *
   * @param subsystem The subsystems.
   * @return A command that blinks the status LEDs red.
   */
  public static Command indicateErrorWithBlink(Subsystems subsystem) {
    return new BlinkColor(subsystem.statusLEDs, RED).withName("IndicateErrorWithBlink");
  }

  /**
   * Returns a command that turns the status LEDs red while a condition is true.
   *
   * @param subsystem The subsystems.
   * @param condition The condition for the command to run.
   * @return A command that turns the status LEDs red.
   */
  public static Command indicateErrorWithSolid(Subsystems subsystem) {
    return Commands.sequence(
            setColor(subsystem.statusLEDs, RED), Commands.idle(subsystem.statusLEDs))
        .withName("IndicateErrorWithSolid");
  }
}
