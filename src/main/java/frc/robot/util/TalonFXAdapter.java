/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.units.measure.Voltage;

/** A motor controller implementation based on the CTR Electronics TalonFX controller. */
public final class TalonFXAdapter implements MotorController {
  private final TalonFX talonFX;
  private final double metersPerRotation;
  private final MotorOutputConfigs motorOutputConfigs;

  /**
   * Constructs a TalonFXAdapter.
   *
   * <p>This constructor assumes the {@link TalonFX} object is already configured or will be
   * configured to match the provided motor output configuration by the caller.
   *
   * @param talonFX The TalonFX object to adapt.
   * @param motorOutputConfigs The motor output configuration.
   * @param metersPerRotation The distance in meters the attached mechanism moves per rotation of
   *     the output shaft.
   */
  public TalonFXAdapter(
      TalonFX talonFX, MotorOutputConfigs motorOutputConfigs, double metersPerRotation) {
    this.talonFX = talonFX;
    this.motorOutputConfigs = motorOutputConfigs;
    this.metersPerRotation = metersPerRotation;
  }

  /**
   * Constructs a TalonFXAdapter.
   *
   * @param talonFX The TalonFX object to adapt.
   * @param direction The direction the motor rotates when a positive voltage is applied.
   * @param idleMode The motor behavior when idle (i.e. brake or coast mode).
   * @param metersPerRotation The distance in meters the attached mechanism moves per rotation of
   *     the output shaft.
   */
  public TalonFXAdapter(
      TalonFX talonFX, MotorDirection direction, MotorIdleMode idleMode, double metersPerRotation) {
    this(talonFX, new MotorOutputConfigs(), metersPerRotation);

    motorOutputConfigs.NeutralMode = idleMode.forTalonFX();
    motorOutputConfigs.Inverted = direction.forTalonFX();

    applyConfig(talonFX, motorOutputConfigs);
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
    talonFX.getConfigurator().refresh(motorOutputConfigs);
    motorOutputConfigs.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    applyConfig(talonFX, motorOutputConfigs);
    ;
  }

  @Override
  public boolean getInverted() {
    return motorOutputConfigs.Inverted == InvertedValue.Clockwise_Positive;
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    motorOutputConfigs.NeutralMode = idleMode.forTalonFX();
    applyConfig(talonFX, motorOutputConfigs);
    ;
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

    // Get the motor output configuration from the leader and apply it to the follower.
    MotorOutputConfigs followerMotorOutputConfigs = new MotorOutputConfigs();

    followerMotorOutputConfigs.Inverted = motorOutputConfigs.Inverted;
    followerMotorOutputConfigs.NeutralMode = motorOutputConfigs.NeutralMode;
    followerMotorOutputConfigs.DutyCycleNeutralDeadband =
        motorOutputConfigs.DutyCycleNeutralDeadband;
    followerMotorOutputConfigs.PeakForwardDutyCycle = motorOutputConfigs.PeakForwardDutyCycle;
    followerMotorOutputConfigs.PeakReverseDutyCycle = motorOutputConfigs.PeakReverseDutyCycle;
    followerMotorOutputConfigs.ControlTimesyncFreqHz = motorOutputConfigs.ControlTimesyncFreqHz;

    applyConfig(follower, followerMotorOutputConfigs);

    // Configure the follower to follow the leader.
    Follower followerConfig = new Follower(talonFX.getDeviceID(), isInvertedFromLeader);

    follower.setControl(followerConfig);

    return new TalonFXAdapter(follower, followerMotorOutputConfigs, metersPerRotation);
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

  public double getStatorCurrent() {
    return talonFX.getStatorCurrent().refresh().getValueAsDouble();
  }

  public double getTorqueCurrent() {
    return talonFX.getTorqueCurrent().refresh().getValueAsDouble();
  }

  private static void applyConfig(TalonFX talonFX, MotorOutputConfigs motorOutputConfigs) {
    for (int i = 0; i < 5; i++) {
      StatusCode status = talonFX.getConfigurator().apply(motorOutputConfigs);
      if (status.isOK()) {
        return;
      }
      System.out.println(
          String.format(
              "ERROR: Failed to apply motor output configs of TalonFX ID %d: %s (%s)",
              talonFX.getDeviceID(), status.getDescription(), status.getName()));
    }
  }
}
