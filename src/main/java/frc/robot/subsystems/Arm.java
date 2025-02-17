/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.ArmParameters;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.MotorUtils;

@RobotPreferencesLayout(groupName = "Arm", row = 1, column = 0, width = 1, height = 1)
public class Arm extends SubsystemBase implements ActiveSubsystem, ShuffleboardProducer {
  private static final double ERROR_MARGIN = Math.toRadians(2);
  private static final double ERROR_TIME = 1.0;

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Arm", "Enable Tab", false);

  private static final DataLog LOG = DataLogManager.getLog();

  private final double MIN_ANGLE;
  private final double MAX_ANGLE;

  private final TalonFX motor;
  private final DutyCycleEncoder absoluteEncoder;
  private final Timer stuckTimer = new Timer();
  private double currentAngle = 0;
  private double currentAbsoluteAngle = 0;
  private double currentVelocity = 0;
  private double goalAngle = 0;
  private boolean enabled;
  private boolean hasError = false;

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private DoubleLogEntry logCurrentAngle;
  private DoubleLogEntry logCurrentAbsoluteAngle;
  private DoubleLogEntry logCurrentVelocity;
  private DoubleLogEntry logGoalAngle;
  private BooleanLogEntry logEnabled;

  /** Creates a new Arm. */
  public Arm(ArmParameters parameters) {
    setName(parameters.toString());
    MIN_ANGLE = parameters.getMinAngleRad();
    MAX_ANGLE = parameters.getMaxAngleRad();

    absoluteEncoder =
        new DutyCycleEncoder(
            parameters.getEncoderID(), 2 * Math.PI, parameters.getAbsoluteEncoderZeroOffset());
    absoluteEncoder.setInverted(parameters.isAbsoluteEncoderInverted());
    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);

    motor = new TalonFX(parameters.getMotorID(), "rio");
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    FeedbackConfigs feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = parameters.getGearRatio();
    // set slot 0 gains
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = parameters.getkS();
    // Need to convert kV and kA from radians to rotations.
    slot0Configs.kV = parameters.getkV() * 2 * Math.PI;
    slot0Configs.kA = parameters.getkA() * 2 * Math.PI;
    slot0Configs.kG = 0.9;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = 80.0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    // set Motion Magic settings
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        0.3 * parameters.getMaxAngularSpeed() / (2 * Math.PI);
    motionMagicConfigs.MotionMagicAcceleration =
        0.3 * parameters.getMaxAngularAcceleration() / (2 * Math.PI);

    TalonFXConfigurator configurator = motor.getConfigurator();

    configurator.apply(talonFXConfigs);
    configurator.setPosition(absoluteEncoder.get() / (2 * Math.PI));

    logCurrentAngle =
        new DoubleLogEntry(LOG, String.format("/%s/Current Angle", parameters.name()));
    logCurrentAbsoluteAngle =
        new DoubleLogEntry(LOG, String.format("/%s/Current Absolute Angle", parameters.name()));
    logCurrentVelocity =
        new DoubleLogEntry(LOG, String.format("/%s/Current Velocity", parameters.name()));
    logGoalAngle = new DoubleLogEntry(LOG, String.format("/%s/Goal Angle", parameters.name()));
    logEnabled = new BooleanLogEntry(LOG, String.format("/%s/Enabled", parameters.name()));
  }

  /** Updates the sensor state. */
  private void updateSensorState() {
    currentAngle = motor.getPosition().refresh().getValueAsDouble() * 2 * Math.PI;
    currentVelocity = motor.getVelocity().refresh().getValueAsDouble() * 2 * Math.PI;
    currentAbsoluteAngle = absoluteEncoder.get();
    checkError();

    logCurrentAngle.append(currentAngle);
    logCurrentVelocity.append(currentVelocity);
    logCurrentAbsoluteAngle.append(currentAbsoluteAngle);
  }

  /**
   * Checks if the arm is beyond its maximum and minimum angles by 2.0 degrees or is taking more
   * than 1.0 second to be within 2.0 degrees of the goal angle.
   */
  private void checkError() {
    if (MathUtil.isNear(goalAngle, currentAngle, ERROR_MARGIN)) {
      stuckTimer.stop();
      stuckTimer.reset();
    } else {
      if (!stuckTimer.isRunning()) {
        stuckTimer.restart();
      }
    }

    hasError =
        currentAngle > MAX_ANGLE + ERROR_MARGIN
            || currentAngle < MIN_ANGLE - ERROR_MARGIN
            || stuckTimer.hasElapsed(ERROR_TIME);
  }

  /** Sets the goal angle in radians and enables periodic control. */
  public void setGoalAngle(double angle) {
    angle = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    goalAngle = angle;
    enabled = true;
    // set target position to 100 rotations
    motor.setControl(motionMagicRequest.withPosition(angle / (2 * Math.PI)));
    logGoalAngle.append(angle);
    logEnabled.update(enabled);
  }

  /** Returns whether the coral arm is at goal angle. */
  public boolean atGoalAngle() {
    return Math.abs(goalAngle - currentAngle) <= Math.toRadians(1);
  }

  /** Returns whether the coral arm has an error. */
  public boolean hasError() {
    return hasError;
  }

  /** Disables periodic control. */
  @Override
  public void disable() {
    enabled = false;
    logEnabled.update(enabled);
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    MotorUtils.setIdleMode(motor, idleMode);
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
