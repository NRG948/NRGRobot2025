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
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfigAccessor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

/** A motor controller implementation based on the REV Robotics Spark controllers. */
public final class SparkAdapter implements MotorController {
  private interface Accessor {
    /** Returns the motor controller. */
    SparkBase get();

    /** Returns the controller's configuration accessor. */
    SparkBaseConfigAccessor getConfigAccessor();

    /** Returns a new controller-specific configuration. */
    SparkBaseConfig newConfig();

    /** Gets the controller-specific configuration. */
    SparkBaseConfig getConfig();

    /**
     * Returns a new adapter of the same controller type.
     *
     * @param deviceID The device ID of the new controller.
     */
    SparkAdapter newAdapter(int deviceID);
  }

  private static final class SparkMaxAccessor implements Accessor {
    private final SparkMax spark;
    private final SparkMaxConfig config = new SparkMaxConfig();

    private SparkMaxAccessor(SparkMax spark) {
      this.spark = spark;
    }

    @Override
    public SparkBase get() {
      return spark;
    }

    @Override
    public SparkBaseConfigAccessor getConfigAccessor() {
      return spark.configAccessor;
    }

    @Override
    public SparkBaseConfig getConfig() {
      return config;
    }

    @Override
    public SparkBaseConfig newConfig() {
      return new SparkMaxConfig();
    }

    @Override
    public SparkAdapter newAdapter(int deviceID) {
      return new SparkAdapter(new SparkMax(deviceID, spark.getMotorType()));
    }
  }

  private static final class SparkFlexAccessor implements Accessor {
    private final SparkFlex spark;
    private final SparkFlexConfig config = new SparkFlexConfig();

    private SparkFlexAccessor(SparkFlex spark) {
      this.spark = spark;
    }

    @Override
    public SparkBase get() {
      return spark;
    }

    @Override
    public SparkBaseConfigAccessor getConfigAccessor() {
      return spark.configAccessor;
    }

    @Override
    public SparkBaseConfig getConfig() {
      return config;
    }

    @Override
    public SparkBaseConfig newConfig() {
      return new SparkFlexConfig();
    }

    @Override
    public SparkAdapter newAdapter(int deviceID) {
      return new SparkAdapter(new SparkFlex(deviceID, spark.getMotorType()));
    }
  }

  private final Accessor spark;

  /**
   * Constructs a SparkAdapter for a {@link SparkMax} motor controller.
   *
   * <p>This private constructor is intended for use only by delegating public constructors or
   * factory methods. It assumes the SparkMax object is already configured or will be configured
   * approriately by the public constructors or factory methods.
   *
   * @param spark The SparkMax object to adapt.
   */
  private SparkAdapter(SparkMax spark) {
    this.spark = new SparkMaxAccessor(spark);
  }

  /**
   * Constructs a SparkAdapter for a {@link SparkMax} motor controller.
   *
   * @param spark The SparkMax object to adapt.
   * @param isInverted Whether the motor should be inverted.
   * @param brakeMode Whether the motor should brake when stopped.
   * @param metersPerRotation The distance in meters the attached mechanism moves per rotation of
   *     the output shaft.
   */
  public SparkAdapter(
      SparkMax sparkMax, boolean isInverted, boolean brakeMode, double metersPerRotation) {
    this(sparkMax);

    configure(isInverted, brakeMode, metersPerRotation);
  }

  /**
   * Constructs a SparkAdapter for a {@link SparkFlex} motor controller.
   *
   * <p>This private constructor is intended for use only by delegating public constructors or
   * factory methods. It assumes the SparkFlex object is already configured or will be configured
   * approriately by the public constructors or factory methods.
   *
   * @param spark The SparkFlex object to adapt.
   */
  private SparkAdapter(SparkFlex spark) {
    this.spark = new SparkFlexAccessor(spark);
  }

  /**
   * Constructs a SparkAdapter for a {@link SparkFlex} motor controller.
   *
   * @param spark The SparkFlex object to adapt.
   * @param isInverted Whether the motor should be inverted.
   * @param brakeMode Whether the motor should brake when stopped.
   * @param metersPerRotation The distance in meters the attached mechanism moves per rotation of
   *     the output shaft.
   */
  public SparkAdapter(
      SparkFlex sparkFlex, boolean isInverted, boolean brakeMode, double metersPerRotation) {
    this(sparkFlex);

    configure(isInverted, brakeMode, metersPerRotation);
  }

  /**
   * Configures the motor controller.
   *
   * @param isInverted Whether the motor should be inverted.
   * @param brakeMode Whether the motor should brake when stopped.
   * @param metersPerRotation The distance in meters the attached mechanism moves per rotation of
   *     the output shaft.
   */
  private void configure(boolean isInverted, boolean brakeMode, double metersPerRotation) {
    SparkBaseConfig driveMotorConfig = spark.getConfig();

    driveMotorConfig.inverted(isInverted).idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    driveMotorConfig
        .encoder
        .positionConversionFactor(metersPerRotation)
        .velocityConversionFactor(metersPerRotation);

    spark
        .get()
        .configure(
            driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void set(double speed) {
    spark.get().set(speed);
  }

  @Override
  public double get() {
    return spark.get().get();
  }

  @Override
  public void setVoltage(Voltage outputVoltage) {
    spark.get().setVoltage(outputVoltage);
  }

  @Override
  public void setInverted(boolean isInverted) {
    SparkBaseConfig config = spark.getConfig();

    config.inverted(isInverted);

    spark.get().configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public boolean getInverted() {
    return spark.getConfigAccessor().getInverted();
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    SparkBaseConfig config = spark.getConfig();

    config.idleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);

    spark
        .get()
        .configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void disable() {
    spark.get().disable();
  }

  @Override
  public void stopMotor() {
    spark.get().stopMotor();
  }

  @Override
  public MotorController createFollower(int deviceID, boolean isInvertedFromLeader) {
    SparkAdapter follower = spark.newAdapter(deviceID);
    SparkBaseConfigAccessor configAccessor = spark.getConfigAccessor();
    SparkBaseConfig motorOutputConfigs = spark.newConfig();

    // Get the motor output configuration from the leader and apply it to the
    // follower.
    motorOutputConfigs
        .inverted(configAccessor.getInverted())
        .idleMode(configAccessor.getIdleMode());

    motorOutputConfigs
        .encoder
        .positionConversionFactor(configAccessor.encoder.getPositionConversionFactor())
        .velocityConversionFactor(configAccessor.encoder.getVelocityConversionFactor());

    // Configure the follower to follow the leader.
    motorOutputConfigs.follow(spark.get(), isInvertedFromLeader);

    follower
        .spark
        .get()
        .configure(
            motorOutputConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    return follower;
  }

  @Override
  public RelativeEncoder getEncoder() {
    return new SparkEncoderAdapter(spark.get().getEncoder());
  }

  @Override
  public LimitSwitch getForwardLimitSwitch() {
    return new SparkLimitSwitchAdapter(spark.get().getForwardLimitSwitch());
  }

  @Override
  public LimitSwitch getReverseLimitSwitch() {
    return new SparkLimitSwitchAdapter(spark.get().getReverseLimitSwitch());
  }
}
