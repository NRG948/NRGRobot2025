/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.ArmParameters;

@RobotPreferencesLayout(groupName = "Arm", row = 5, column = 0, width = 1, height = 1)
public class Arm extends SubsystemBase implements ShuffleboardProducer {
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Arm", "Enable Tab", false);

  /** The angle of the absolute encoder in radians from the designated zero point. */
  private final double absoluteEncoderOffset;

  private final ArmParameters parameters;
  private final TalonFX motor;
  private final DutyCycleEncoder absoluteEncoder;
  private double currentAngle = 0;
  private double currentAbsoluteAngle = 0;
  private double currentVelocity = 0;
  private double goalAngle = 0;
  private boolean enabled;
  private final double rotationsPerRadians;

  /** Creates a new Arm. */
  public Arm(ArmParameters parameters) {
    setName(parameters.toString());
    this.parameters = parameters;
    rotationsPerRadians = parameters.getGearRatio() / (2 * Math.PI);

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    motor = new TalonFX(parameters.getMotorID());
    absoluteEncoder = new DutyCycleEncoder(parameters.getEncoderID());

    MotorOutputConfigs motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    // set slot 0 gains
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = parameters.getkS();
    /** Need to convert kV and kA from radians to rotations. */
    slot0Configs.kV = parameters.getkV() * 2 * Math.PI;
    slot0Configs.kA = parameters.getkA() * 2 * Math.PI;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    // set Motion Magic Expo settings
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 0;
    motionMagicConfigs.MotionMagicExpo_kV = slot0Configs.kV;
    motionMagicConfigs.MotionMagicExpo_kA = slot0Configs.kA;

    motor.getConfigurator().apply(talonFXConfigs);

    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
    absoluteEncoderOffset =
        absoluteEncoder.get() * (2 * Math.PI) - parameters.getAbsoluteEncoderZeroOffset();
  }

  /** Updates the sensor state. */
  private void updateSensorState() {
    currentAngle =
        motor.getPosition().refresh().getValueAsDouble() / rotationsPerRadians
            + absoluteEncoderOffset;
    currentVelocity = motor.getVelocity().refresh().getValueAsDouble() / rotationsPerRadians;
    currentAbsoluteAngle =
        absoluteEncoder.get() * (2 * Math.PI) - parameters.getAbsoluteEncoderZeroOffset();
  }

  /** Sets the goal angle in radians and enables periodic control. */
  public void setGoalAngle(double angle) {
    goalAngle = angle;
    enabled = true;
    // create a Motion Magic request, voltage output
    final MotionMagicVoltage request = new MotionMagicVoltage(0);
    // set target position to 100 rotations
    motor.setControl(request.withPosition(angle * rotationsPerRadians));
  }

  /** Returns whether the coral arm is at goal angle. */
  public boolean atGoalAngle() {
    return Math.abs(goalAngle - currentAngle) <= Math.toRadians(1);
  }

  /** Disables periodic control. */
  @Override
  public void disable() {
    enabled = false;
  }

  @Override
  public void periodic() {
    updateSensorState();
  }

  @Override
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }
    ShuffleboardTab armTab = Shuffleboard.getTab(getName());
    ShuffleboardLayout statusLayout =
        armTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    statusLayout.addBoolean("Enabled", () -> enabled);
    statusLayout.addDouble("Current Angle of Motor Encoder", () -> Math.toDegrees(currentAngle));
    statusLayout.addDouble(
        "Current Angle of Absolute Encoder", () -> Math.toDegrees(currentAbsoluteAngle));
    statusLayout.addDouble("Goal Angle", () -> Math.toDegrees(goalAngle));
    statusLayout.addDouble("Current Velocity", () -> Math.toDegrees(currentVelocity));
    ShuffleboardLayout controlLayout =
        armTab.getLayout("Control", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
    GenericEntry angle = controlLayout.add("Angle", 0).getEntry();
    controlLayout.add(
        Commands.sequence(
                Commands.runOnce(() -> setGoalAngle(Math.toRadians(angle.getDouble(0))), this),
                Commands.idle(this).until(this::atGoalAngle))
            .withName("Set Angle"));
    controlLayout.add(Commands.runOnce(this::disable, this).withName("Disable"));
  }
}
