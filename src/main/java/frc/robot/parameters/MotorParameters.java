/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.parameters;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.MotorController;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.SparkAdapter;
import frc.robot.util.TalonFXAdapter;
import org.ejml.simple.UnsupportedOperation;

/** A enum representing the properties on a specific motor type. */
public enum MotorParameters {
  /**
   * A VEX PRO <a href="https://www.vexrobotics.com/217-6515.html">Falcon 500</a> motor with
   * integrated Talon FX motor controller and encoders.
   */
  Falcon500(DCMotor.getFalcon500(1)),

  /**
   * A WestCoast Products <a href="https://docs.wcproducts.com/kraken-x60/kraken-x60-motor">Kraken
   * X60</a> motor with integrated Talon FX motor controller and encoders.
   */
  KrakenX60(DCMotor.getKrakenX60(1)),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1650/">NEO Brushless Motor V1.1</a>
   * with integrated encoder.
   */
  NeoV1_1(DCMotor.getNEO(1)),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1652/">NEO Vortex Brushless
   * Motor</a> with <a href="https://www.revrobotics.com/rev-11-2158/">SparkMax</a> motor
   * controller.
   */
  NeoVortexMax(DCMotor.getNeoVortex(1)),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1652/">NEO Vortex Brushless
   * Motor</a> with <a href="https://www.revrobotics.com/rev-11-2159/">SparkFlex</a> motor
   * controller.
   */
  NeoVortexFlex(DCMotor.getNeoVortex(1)),

  /**
   * A REV Robotics <a href="https://www.revrobotics.com/rev-21-1651/">NEO 550 Brushless Motor</a>
   * with integrated encoder.
   */
  Neo550(DCMotor.getNeo550(1));

  private final DCMotor motor;

  /**
   * Constructs an instance of this enum.
   *
   * @param freeSpeedRPM The free speed RPM.
   * @param stallTorque The stall torque in Nm.
   * @param pulsesPerRevolution The number of pulses per revolution of the motor reported by the
   *     integrated encoder.
   */
  MotorParameters(DCMotor motor) {
    this.motor = motor;
  }

  /**
   * Returns the free speed RPM of the motor.
   *
   * @return The free speed RPM.
   */
  public double getFreeSpeedRPM() {
    // return this.freeSpeedRPM;
    return this.motor.freeSpeedRadPerSec / (2 * Math.PI) * 60;
  }

  public DCMotor getDCMotor() {
    return motor;
  }

  /**
   * @return The stall torque in Nm.
   */
  public double getStallTorque() {
    return this.motor.stallTorqueNewtonMeters;
  }

  /**
   * Returns a new {@link MotorController} implementation for this motor type.
   *
   * @param deviceID The CAN device ID.
   * @param isInverted Whether the motor should be inverted.
   * @param idleMode The motor behavior when idle (i.e. brake or coast mode).
   * @param metersPerRotation The distance in meters the attached mechanism moves per rotation of
   *     the output shaft.
   * @return A new MotorController implementation.
   */
  public MotorController newController(
      int deviceID, boolean isInverted, MotorIdleMode idleMode, double metersPerRotation) {
    switch (this) {
      case Falcon500:
      case KrakenX60:
        return new TalonFXAdapter(new TalonFX(deviceID), isInverted, idleMode, metersPerRotation);

      case NeoV1_1:
      case NeoVortexMax:
      case Neo550:
        return new SparkAdapter(
            new SparkMax(deviceID, MotorType.kBrushless), isInverted, idleMode, metersPerRotation);

      case NeoVortexFlex:
        return new SparkAdapter(
            new SparkFlex(deviceID, MotorType.kBrushless), isInverted, idleMode, metersPerRotation);

      default:
        throw new UnsupportedOperation("Unknown Motor Type");
    }
  }
}
