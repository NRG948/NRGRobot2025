/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CAN.TalonFX.COMPETITION_BOT_CLIMBER_MOTOR_ID;
import static frc.robot.Constants.RobotConstants.CAN.TalonFX.PRACTICE_BOT_CLIMBER_MOTOR_ID;
import static frc.robot.Constants.RobotConstants.DigitalIO.CLIMBER_ABSOLUTE_ENCODER;
import static frc.robot.RobotContainer.RobotSelector.CompetitionRobot2025;
import static frc.robot.RobotContainer.RobotSelector.PracticeRobot2025;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.hardware.TalonFX;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferences.DoubleValue;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.AbsoluteAngleEncoder;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.RelativeEncoder;
import frc.robot.util.RevThroughboreEncoderAdapter;
import frc.robot.util.TalonFXAdapter;
import java.util.Map;

@RobotPreferencesLayout(groupName = "Climber", row = 0, column = 6, width = 2, height = 3)
public class Climber extends SubsystemBase implements ShuffleboardProducer, ActiveSubsystem {
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Climber", "Enable Tab", false);

  private static final DataLog LOG = DataLogManager.getLog();

  private final double MIN_ANGLE = Math.toRadians(-94);
  private final double MAX_ANGLE = Math.toRadians(110);

  @SuppressWarnings("unused")
  private final double GEAR_RATIO = 5.0 * 5.0 * 66.0 / 18.0;

  /**
   * The robot's climber parameters.
   *
   * @param motorID The CAN ID of the climber motor.
   * @param encoderZeroOffset The zero point of the encoder in radians with range -π to π radians.
   *     <p>To get this value, set it to zero initially and read the value from the encoder. Pass
   *     the observed value here.
   */
  public record ClimberParameters(int motorID, double encoderZeroOffset) {}

  public static final ClimberParameters PRACTICE_BOT_PARAMETERS =
      new ClimberParameters(PRACTICE_BOT_CLIMBER_MOTOR_ID, Math.toRadians(173.1));
  public static final ClimberParameters COMPETITION_BOT_PARAMETERS =
      new ClimberParameters(COMPETITION_BOT_CLIMBER_MOTOR_ID, Math.toRadians(179.74));
  public static final ClimberParameters PARAMETERS =
      RobotContainer.ROBOT_TYPE
          .select(
              Map.of(
                  PracticeRobot2025, PRACTICE_BOT_PARAMETERS,
                  CompetitionRobot2025, COMPETITION_BOT_PARAMETERS))
          .orElse(COMPETITION_BOT_PARAMETERS);

  @RobotPreferencesValue
  public static DoubleValue CLIMB_MAX_POWER = new DoubleValue("Climber", "Max Power", 0.75);

  @RobotPreferencesValue
  public static DoubleValue TOLERANCE_DEG = new DoubleValue("Climber", "Tolerance (deg)", 1);

  @RobotPreferencesValue
  public static DoubleValue PROPORTIONAL_CONTROL_THRESHOLD_DEG =
      new DoubleValue("Climber", "Proportional Control Threshold", 10);

  private double currentAngle; // in radians
  private double goalAngle; // in radians
  private boolean enabled;

  private final AbsoluteAngleEncoder absoluteEncoder =
      new RevThroughboreEncoderAdapter(
          CLIMBER_ABSOLUTE_ENCODER, true, PARAMETERS.encoderZeroOffset);

  private TalonFXAdapter mainMotor =
      new TalonFXAdapter(
          "/Climber",
          new TalonFX(PARAMETERS.motorID, "rio"),
          COUNTER_CLOCKWISE_POSITIVE,
          BRAKE,
          1.0); // filler value; motor encoder value is not used.

  private DoubleLogEntry logCurrentAbsoluteAngle =
      new DoubleLogEntry(LOG, "/Climber/Absolute Angle");
  private DoubleLogEntry logGoalAngle = new DoubleLogEntry(LOG, "/Climber/Goal Angle");
  private BooleanLogEntry logEnabled = new BooleanLogEntry(LOG, "/Climber/Is Enabled");
  private DoubleLogEntry logMotorPower = new DoubleLogEntry(LOG, "/Climber/Motor Power");
  private DoubleLogEntry logMotorAngularVelocity = new DoubleLogEntry(LOG, "/Climber/Motor Rot/s");

  private RelativeEncoder encoder = mainMotor.getEncoder();

  /** Creates a new Climber subsystem. */
  public Climber() {}

  @Override
  public void periodic() {
    updateTelemetry();

    double angleError = goalAngle - currentAngle;
    double motorPower;
    if (enabled && !atGoalAngle()) {
      // Runs at max power until within small angle of goal, then ramps power down linearly.
      double maxPower = CLIMB_MAX_POWER.getValue();
      double kP = maxPower / Math.toRadians(PROPORTIONAL_CONTROL_THRESHOLD_DEG.getValue());
      motorPower = MathUtil.clamp(kP * angleError, -maxPower, maxPower);
    } else {
      motorPower = 0;
    }
    mainMotor.set(motorPower);
    logMotorPower.append(motorPower);
  }

  /** Sets the goal angle in radians and enables periodic control. */
  public void setGoalAngle(double angle) {
    angle = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    goalAngle = angle;
    enabled = true;

    logGoalAngle.append(angle);
    logEnabled.update(enabled);
  }

  public void disable() {
    enabled = false;
    logEnabled.update(enabled);
  }

  public boolean atGoalAngle() {
    return MathUtil.isNear(goalAngle, currentAngle, Math.toRadians(TOLERANCE_DEG.getValue()));
  }

  public double getCurrentAngle() {
    return currentAngle;
  }

  private void updateTelemetry() {
    currentAngle = absoluteEncoder.getAngle(); // in rad

    logCurrentAbsoluteAngle.append(currentAngle);
    logMotorAngularVelocity.append(encoder.getVelocity()); // we set meter/rot = 1
    mainMotor.logTelemetry();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    if (RobotContainer.isCompBot()) {
      mainMotor.setIdleMode(idleMode);
    }
  }

  @Override
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }

    ShuffleboardTab climberTab = Shuffleboard.getTab(getName());

    ShuffleboardLayout statusLayout =
        climberTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    statusLayout.addBoolean("Enabled", () -> enabled);
    statusLayout.addDouble("Current Angle", () -> Math.toDegrees(currentAngle));
    statusLayout.addDouble("Goal Angle", () -> Math.toDegrees(goalAngle));

    ShuffleboardLayout controlLayout =
        climberTab.getLayout("Control", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
    GenericEntry angle = controlLayout.add("Angle (deg)", 0).getEntry();
    controlLayout.add(
        Commands.sequence(
                Commands.runOnce(() -> setGoalAngle(Math.toRadians(angle.getDouble(0))), this),
                Commands.idle(this).until(this::atGoalAngle))
            .withName("Set Angle (deg)"));
    controlLayout.add(Commands.runOnce(this::disable, this).withName("Disable"));
  }
}
