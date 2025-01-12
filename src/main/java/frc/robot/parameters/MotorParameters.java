/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.util.MotorController;
import frc.robot.util.SparkAdapter;
import frc.robot.util.TalonFXAdapter;
import org.ejml.simple.UnsupportedOperation;

/** A enum representing the properties on a specific motor type. */
public enum MotorParameters {
  /**
   * A VEX PRO <a href="https://www.vexrobotics.com/217-6515.html">Falcon 500</a> motor with
   * integrated Talon FX motor controller and encoders.
   */
  Falcon500(6380.0, 4.69, 2048),
  /**
   * A WestCoast Products <a href="https://docs.wcproducts.com/kraken-x60/kraken-x60-motor">Kraken
   * X60</a> motor with integrated Talon FX motor controller and encoders.
   */
  KrakenX60(6000.0, 7.09, 2048),
  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1650/">NEO Brushless Motor V1.1</a>
   * with integrated encoder.
   */
  NeoV1_1(5676.0, 3.75, 42),
  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1652/">NEO Vortex Brushless
   * Motor</a> with <a href="https://www.revrobotics.com/rev-11-2158/">SparkMax</a> motor
   * controller.
   */
  NeoVortexMax(6784.0, 3.6, 42),
  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1652/">NEO Vortex Brushless
   * Motor</a> with <a href="https://www.revrobotics.com/rev-11-2159/">SparkFlex</a> motor
   * controller.
   */
  NeoVortexFlex(6784.0, 3.6, 7168),
  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1651/">NEO 550 Brushless Motor</a>
   * with integrated encoder.
   */
  Neo550(11000.0, 0.97, 42);

  private double freeSpeedRPM;
  private double stallTorque;
  private int pulsesPerRevolution;

  /**
   * Constructs an instance of this enum.
   *
   * @param freeSpeedRPM The free speed RPM.
   * @param stallTorque The stall torque in Nm.
   * @param pulsesPerRevolution The number of pulses per revolution of the motor reported by the
   *     integrated encoder.
   */
  MotorParameters(double freeSpeedRPM, double stallTorque, int pulsesPerRevolution) {
    this.freeSpeedRPM = freeSpeedRPM;
    this.stallTorque = stallTorque;
    this.pulsesPerRevolution = pulsesPerRevolution;
  }

  /**
   * Returns the free speed RPM of the motor.
   *
   * @return The free speed RPM.
   */
  public double getFreeSpeedRPM() {
    return this.freeSpeedRPM;
  }

  /**
   * Returns the stall torque of the motor in Nm.
   *
   * @return The stall torque in Nm.
   */
  public double getStallTorque() {
    return this.stallTorque;
  }

  /**
   * Returns the number of pulses per revolution of the integrated encoder.
   *
   * @return The number of pulses per revolution of the integrated encoder.
   */
  public int getPulsesPerRevolution() {
    return this.pulsesPerRevolution;
  }

  public MotorController getController(
      int deviceID, boolean isInverted, boolean brakeMode, double metersPerRotation) {
    switch (this) {
      case Falcon500:
      case KrakenX60:
        return new TalonFXAdapter(new TalonFX(deviceID), isInverted, brakeMode, metersPerRotation);
      case NeoV1_1:
      case NeoVortexMax:
      case Neo550:
        return new SparkAdapter(
            new SparkMax(deviceID, MotorType.kBrushless), isInverted, brakeMode, metersPerRotation);
      default:
        throw new UnsupportedOperation("Unknown Motor Type");
    }
  }
}
