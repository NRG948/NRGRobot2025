/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXAdapter implements MotorController {
  private final TalonFX talonFX;
  private final double metersPerRotation;

  /**
   * Constructs a TalonFXAdapter.
   *
   * <p>This private constructor is intended for use only by delegating public constructors or
   * factory methods. It assumes the {@link TalonFX} object is already configured or will be
   * configured appropriately by the public constructors or factory methods.
   *
   * @param talonFX The TalonFX object to adapt.
   * @param metersPerRotation The distance in meters the attached mechanism moves per rotation of
   *     the output shaft.
   */
  private TalonFXAdapter(TalonFX talonFX, double metersPerRotation) {
    this.talonFX = talonFX;
    this.metersPerRotation = metersPerRotation;
  }

  /**
   * Constructs a TalonFXAdapter.
   *
   * @param talonFX The TalonFX object to adapt.
   * @param isInverted Whether the motor should be inverted.
   * @param brakeMode Whether the motor should brake when stopped.
   * @param metersPerRotation The distance in meters the attached mechanism moves per rotation of
   *     the output shaft.
   */
  public TalonFXAdapter(
      TalonFX talonFX, boolean isInverted, boolean brakeMode, double metersPerRotation) {
    this(talonFX, metersPerRotation);
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    motorOutputConfigs.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    motorOutputConfigs.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    talonFX.getConfigurator().apply(motorOutputConfigs);
  }

  @Override
  public void set(double speed) {
    talonFX.set(speed);
  }

  @Override
  public double get() {
    return talonFX.get();
  }

  @Override
  public void setVoltage(Voltage outputVoltage) {
    talonFX.setVoltage(outputVoltage.magnitude());
  }

  @Override
  public void setInverted(boolean isInverted) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    talonFX.getConfigurator().refresh(motorOutputConfigs);
    motorOutputConfigs.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    talonFX.getConfigurator().apply(motorOutputConfigs);
  }

  @Override
  public boolean getInverted() {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    talonFX.getConfigurator().refresh(motorOutputConfigs);

    return motorOutputConfigs.Inverted == InvertedValue.Clockwise_Positive;
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    talonFX.getConfigurator().refresh(motorOutputConfigs);
    motorOutputConfigs.NeutralMode = brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    // TODO: Check to see if this will override the inverted mode.
    talonFX.getConfigurator().apply(motorOutputConfigs);
  }

  @Override
  public void disable() {
    talonFX.disable();
  }

  @Override
  public void stopMotor() {
    talonFX.stopMotor();
  }

  @Override
  public MotorController createFollower(int deviceID, boolean isInvertedFromLeader) {
    TalonFX follower = new TalonFX(deviceID, talonFX.getNetwork());

    // Get the motor output configuration from the leader and apply it to the
    // follower.
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    talonFX.getConfigurator().refresh(motorOutputConfigs);
    follower.getConfigurator().apply(motorOutputConfigs);

    // Configure the follower to follow the leader.
    Follower followerConfig = new Follower(talonFX.getDeviceID(), isInvertedFromLeader);

    follower.setControl(followerConfig);

    return new TalonFXAdapter(follower, metersPerRotation);
  }

  @Override
  public RelativeEncoder getEncoder() {
    return new TalonFXEncoderAdapter(talonFX, metersPerRotation);
  }

  @Override
  public LimitSwitch getForwardLimitSwitch() {
    return new TalonFXLimitSwitchAdapter<ForwardLimitValue>(
        talonFX.getForwardLimit(), ForwardLimitValue.ClosedToGround);
  }

  @Override
  public LimitSwitch getReverseLimitSwitch() {
    return new TalonFXLimitSwitchAdapter<ReverseLimitValue>(
        talonFX.getReverseLimit(), ReverseLimitValue.ClosedToGround);
  }
}
