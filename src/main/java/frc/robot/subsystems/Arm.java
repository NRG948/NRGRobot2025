/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.RobotContainer.RobotSelector.CompetitionRobot2025;
import static frc.robot.RobotContainer.RobotSelector.PracticeRobot2025;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.parameters.ArmParameters;
import frc.robot.parameters.CoralArmParameters;
import frc.robot.parameters.CoralGroundIntakeArmParameters;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.TalonFXAdapter;
import java.util.Map;
import java.util.function.BooleanSupplier;

@RobotPreferencesLayout(groupName = "Arm", row = 2, column = 4, width = 1, height = 1)
public class Arm extends SubsystemBase implements ActiveSubsystem, DataPublisher {
  private static final double TOLERANCE = Math.toRadians(1.5);
  private static final double RADIANS_PER_ROTATION = 2 * Math.PI;
  private static final double ERROR_MARGIN = Math.toRadians(5);
  private static final double ERROR_TIME = 1.0;

  @RobotPreferencesValue(row = 0, column = 0, width = 1, height = 1)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Arm", "Enable Tab", false);

  public static final CoralArmParameters CORAL_ARM_PARAMETERS =
      RobotContainer.ROBOT_TYPE
          .select(
              Map.of(
                  PracticeRobot2025, CoralArmParameters.PracticeBase2025,
                  CompetitionRobot2025, CoralArmParameters.CompetitionBase2025))
          .orElse(CoralArmParameters.CompetitionBase2025);

  public static final CoralGroundIntakeArmParameters CORAL_GROUND_INTAKE_ARM_PARAMETERS =
      RobotContainer.ROBOT_TYPE
          .select(
              Map.of(
                  PracticeRobot2025,
                  CoralGroundIntakeArmParameters.PracticeBase2025,
                  CompetitionRobot2025,
                  CoralGroundIntakeArmParameters.CompetitionBase2025))
          .orElse(CoralGroundIntakeArmParameters.CompetitionBase2025);

  private static final DataLog LOG = DataLogManager.getLog();

  private final double minAngle;
  private final double maxAngle;
  private final double stowedAngle;

  private final TalonFX talonFX;
  private final TalonFXAdapter motor;
  private final RelativeEncoder relativeEncoder;
  private final Timer stuckTimer = new Timer();
  private final BooleanSupplier hasCoral;
  private double currentAngle = 0;
  private double currentVelocity = 0;
  private double goalAngle = 0;
  private boolean enabled;
  private boolean hasError = false;

  private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);

  private DoubleLogEntry logCurrentAngle;
  private DoubleLogEntry logCurrentVelocity;
  private DoubleLogEntry logGoalAngle;
  private BooleanLogEntry logEnabled;

  /** Creates a new Arm from {@link CoralArmParameters}. */
  public Arm(CoralArmParameters parameters, BooleanSupplier hasCoral) {
    this((ArmParameters) parameters, hasCoral);
  }

  /** Creates a new Arm from {@link CoralGroundIntakeArmParameters}. */
  public Arm(CoralGroundIntakeArmParameters parameters, BooleanSupplier hasCoral) {
    this((ArmParameters) parameters, hasCoral);
  }

  /** Creates a new Arm. */
  private Arm(ArmParameters parameters, BooleanSupplier hasCoral) {
    setName(parameters.getArmName());
    this.hasCoral = hasCoral;
    minAngle = parameters.getMinAngleRad();
    maxAngle = parameters.getMaxAngleRad();
    stowedAngle = parameters.getStowedAngleRad();

    talonFX = new TalonFX(parameters.getMotorID(), "rio");
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    MotorOutputConfigs motorOutputConfigs = talonFXConfigs.MotorOutput;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    motorOutputConfigs.Inverted = parameters.getMotorDirection().forTalonFX();
    FeedbackConfigs feedbackConfigs = talonFXConfigs.Feedback;
    feedbackConfigs.SensorToMechanismRatio = parameters.getGearRatio();
    // set slot 0 gains
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = parameters.getkS();
    // Need to convert kV and kA from radians to rotations.
    slot0Configs.kV = parameters.getkV() * RADIANS_PER_ROTATION;
    slot0Configs.kA = parameters.getkAWithoutCoral() * RADIANS_PER_ROTATION;
    slot0Configs.kG = 0.9;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kP = 120.0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    // set slot 1 gains
    Slot1Configs slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kS = parameters.getkS();
    // Need to convert kV and kA from radians to rotations.
    slot1Configs.kV = parameters.getkV() * RADIANS_PER_ROTATION;
    slot1Configs.kA = parameters.getkAWithCoral() * RADIANS_PER_ROTATION;
    slot1Configs.kG = 0.9;
    slot1Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot1Configs.kP = 120.0;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0;

    // set Motion Magic settings
    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        parameters.getAngularSpeed() / RADIANS_PER_ROTATION;
    motionMagicConfigs.MotionMagicAcceleration =
        parameters.getAngularAcceleration() / RADIANS_PER_ROTATION;

    TalonFXConfigurator configurator = talonFX.getConfigurator();

    configurator.apply(talonFXConfigs);

    String logPrefix = "/" + parameters.getArmName();

    motor =
        new TalonFXAdapter(logPrefix, talonFX, talonFXConfigs.MotorOutput, RADIANS_PER_ROTATION);
    relativeEncoder = motor.getEncoder();
    relativeEncoder.setPosition(parameters.getStowedAngleRad());

    logCurrentAngle = new DoubleLogEntry(LOG, logPrefix + "/Current Angle");
    logCurrentVelocity = new DoubleLogEntry(LOG, logPrefix + "/Current Velocity");
    logGoalAngle = new DoubleLogEntry(LOG, logPrefix + "/Goal Angle");
    logEnabled = new BooleanLogEntry(LOG, logPrefix + "/Enabled");
  }

  /** Updates and logs the current sensor states. */
  private void updateTelemetry() {
    currentAngle = relativeEncoder.getPosition();
    currentVelocity = relativeEncoder.getVelocity();
    checkError();

    logCurrentAngle.append(currentAngle);
    logCurrentVelocity.append(currentVelocity);
    motor.logTelemetry();
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
        currentAngle > maxAngle + ERROR_MARGIN
            || currentAngle < minAngle - ERROR_MARGIN
            || stuckTimer.hasElapsed(ERROR_TIME);
  }

  /** Sets the goal angle in radians and enables periodic control. */
  public void setGoalAngle(double angle) {
    angle = MathUtil.clamp(angle, minAngle, maxAngle);
    goalAngle = angle;
    enabled = true;

    talonFX.setControl(
        motionMagicRequest
            .withPosition(angle / RADIANS_PER_ROTATION)
            .withSlot(hasCoral.getAsBoolean() ? 1 : 0));

    logGoalAngle.append(angle);
    logEnabled.update(enabled);
  }

  /** Returns whether the coral arm is at goal angle. */
  public boolean atGoalAngle(double goalAngle) {
    return Math.abs(goalAngle - currentAngle) <= TOLERANCE;
  }

  public boolean atGoalAngle() {
    return atGoalAngle(this.goalAngle);
  }

  public boolean isStowed() {
    return atGoalAngle(this.stowedAngle);
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
    // Always leave the arms in brake mode to not flop when manually moving the arm.
    // motor.setIdleMode(idleMode);
  }

  @Override
  public void periodic() {
    updateTelemetry();
  }

  public void publishData() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }
    SmartDashboard.putBoolean("Arm/Status/Enabled", enabled);
    SmartDashboard.putNumber("Arm/Status/Current Angle of Motor Encoder", Math.toDegrees(currentAngle));
    SmartDashboard.putNumber("Arm/Status/Goal Angle", Math.toDegrees(goalAngle));
    SmartDashboard.putNumber("Arm/Status/Current Velocity", Math.toDegrees(currentVelocity));

    SmartDashboard.getNumber("Arm/Control/Angle", 0);
  }
}
