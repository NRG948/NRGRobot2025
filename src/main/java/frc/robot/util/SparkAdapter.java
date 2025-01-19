/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

public class SparkAdapter implements MotorController {
  private final SparkBase spark;

  public SparkAdapter(
      SparkMax spark, boolean isInverted, boolean brakeMode, double metersPerRotation) {
    SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
    driveMotorConfig
        .inverted(isInverted)
        .idleMode(IdleMode.kBrake)
        .encoder
        .positionConversionFactor(metersPerRotation)
        .velocityConversionFactor(metersPerRotation);
    spark.configure(
        driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    this.spark = spark;
  }

  @Override
  public void set(double speed) {
    spark.set(speed);
  }

  @Override
  public double get() {
    return spark.get();
  }

  @Override
  public void setVoltage(Voltage outputVoltage) {
    spark.setVoltage(outputVoltage);
  }

  @Override
  public void setInverted(boolean isInverted) {
    spark.setInverted(isInverted);
  }

  @Override
  public boolean getInverted() {
    return spark.getInverted();
  }

  @Override
  public void disable() {
    spark.disable();
  }

  @Override
  public void stopMotor() {
    spark.stopMotor();
  }

  @Override
  public RelativeEncoder getEncoder() {
    return new SparkEncoderAdapter(spark.getEncoder());
  }

  @Override
  public LimitSwitch getForwardLimitSwitch() {
    return new SparkLimitSwitchAdapter(spark.getForwardLimitSwitch());
  }

  @Override
  public LimitSwitch getReverseLimitSwitch() {
    return new SparkLimitSwitchAdapter(spark.getReverseLimitSwitch());
  }
}
