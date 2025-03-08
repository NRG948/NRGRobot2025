/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import static frc.robot.commands.CoralCommands.CORAL_DETECTION_DELAY;
import static frc.robot.parameters.Colors.GREEN;
import static frc.robot.parameters.Colors.RED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.parameters.Colors;
import frc.robot.subsystems.AlgaeGrabber;
import frc.robot.subsystems.CoralRoller;
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
   * @param subsystems The subsystems container.
   * @param period The period of the blink.
   * @return A command that blinks the status LEDs.
   */
  public static Command indicateCoralAcquired(Subsystems subsystems) {
    StatusLED statusLEDs = subsystems.statusLEDs;
    CoralRoller coralRoller = subsystems.coralRoller;

    return Commands.sequence(
            Commands.waitSeconds(CORAL_DETECTION_DELAY),
            new BlinkColor(statusLEDs, GREEN).withTimeout(BLINK_DURATION),
            setColor(statusLEDs, GREEN),
            Commands.idle(statusLEDs).until(() -> !coralRoller.hasCoral()))
        .withName("IndicateCoralAcquired");
  }

  /**
   * Returns a command that blinks the status LEDs green for one second and then sets the color to
   * solid green when an algae is acquired.
   *
   * @param subsystem The subsystems container.
   * @param period The period of the blink.
   * @return A command that blinks the status LEDs.
   */
  public static Command indicateAlgaeAcquired(Subsystems subsystems) {
    return subsystems
        .algaeGrabber
        .map((algaeGrabber) -> indicateAlgaeAcquired(algaeGrabber, subsystems.statusLEDs))
        .orElse(Commands.none());
  }

  /**
   * Returns a command that blinks the status LEDs green for one second and then sets the color to
   * solid green when an algae is acquired.
   *
   * @param algaeGrabber The algae grabber subsystem.
   * @param statusLEDs The status LED subsystem.
   * @param period The period of the blink.
   * @return A command that blinks the status LEDs.
   */
  private static Command indicateAlgaeAcquired(AlgaeGrabber algaeGrabber, StatusLED statusLEDs) {
    return Commands.sequence(
            new BlinkColor(statusLEDs, GREEN).withTimeout(BLINK_DURATION),
            setColor(statusLEDs, GREEN),
            Commands.idle(statusLEDs).until(() -> !algaeGrabber.hasAlgae()))
        .withName("IndicateAlgaeAcquired");
  }

  /**
   * Returns a command that blinks the status LEDs red while a condition is true.
   *
   * @param subsystems The subsystems container.
   * @return A command that blinks the status LEDs red.
   */
  public static Command indicateErrorWithBlink(Subsystems subsystems) {
    return new BlinkColor(subsystems.statusLEDs, RED).withName("IndicateErrorWithBlink");
  }

  /**
   * Returns a command that turns the status LEDs red while a condition is true.
   *
   * @param subsystems The subsystems.
   * @param condition The condition for the command to run.
   * @return A command that turns the status LEDs red.
   */
  public static Command indicateErrorWithSolid(Subsystems subsystems) {
    StatusLED statusLEDs = subsystems.statusLEDs;

    return Commands.sequence(
            setColor(statusLEDs, RED), //
            Commands.idle(statusLEDs))
        .withName("IndicateErrorWithSolid");
  }
}
