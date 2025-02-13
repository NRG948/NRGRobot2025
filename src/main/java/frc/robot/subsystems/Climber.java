/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.util.MotorController;
import frc.robot.util.TalonFXAdapter;

@RobotPreferencesLayout(groupName = "Climber", row = 0, column = 7, width = 1, height = 1)
public class Climber extends SubsystemBase implements ShuffleboardProducer, ActiveSubsystem {
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("Climber", "Enable Tab", false);

  private static final DataLog LOG = DataLogManager.getLog();

  private final double MIN_ANGLE = -89;
  private final double MAX_ANGLE = 150; // TODO: verify real max angle
  private final double GEAR_RATIO = 5 * 5 * 66 / 18;
  private final double ABSOLUTE_ENCODER_ZERO_OFFSET =
      Math.toRadians(
          360 - 173.1); // 360 - needed bcs abs encoder inversion is applied before offset

  @RobotPreferencesValue
  public static DoubleValue CLIMB_POWER = new DoubleValue("Climber", "Climb Power", 0.3);

  @RobotPreferencesValue
  public static DoubleValue NO_LOAD_POWER = new DoubleValue("Climber", "No Load Power", 0.3);

  @RobotPreferencesValue
  public static DoubleValue TOLERANCE_DEG = new DoubleValue("Climber", "Tolerance (deg)", 3);

  @RobotPreferencesValue
  public static DoubleValue CLIMB_GOAL_ANGLE_DEG =
      new DoubleValue("Climber", "Climb Goal Angle (deg)", -88);

  private double currentAngle;
  private double goalAngle;
  private boolean enabled;
  private boolean isClimbing = false;

  private final DutyCycleEncoder absoluteEncoder;

  private MotorController mainMotor =
      new TalonFXAdapter(
          new TalonFX(RobotConstants.CAN.TalonFX.CLIMBER_MAIN_MOTOR_ID, "rio"),
          false,
          true,
          1); // filler value; motor encoder value is not used.

  @SuppressWarnings("unused")
  private MotorController follower =
      mainMotor.createFollower(RobotConstants.CAN.TalonFX.CLIMBER_FOLLOWER_MOTOR_ID, false);

  private DoubleLogEntry logCurrentAbsoluteAngle =
      new DoubleLogEntry(LOG, "Climber/Current Absolute Angle");
  private DoubleLogEntry logGoalAngle = new DoubleLogEntry(LOG, "Climber/Goal Angle");
  private BooleanLogEntry logEnabled = new BooleanLogEntry(LOG, "Climber/Is Enabled");
  private DoubleLogEntry logMotorPower = new DoubleLogEntry(LOG, "Climber/Motor Power");

  /** Creates a new Climber. */
  public Climber() {
    absoluteEncoder =
        new DutyCycleEncoder(
            RobotConstants.DigitalIO.CLIMBER_ABSOLUTE_ENCODER,
            2 * Math.PI,
            ABSOLUTE_ENCODER_ZERO_OFFSET);
    absoluteEncoder.setInverted(true);
    absoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
  }

  @Override
  public void periodic() {
    updateSensorState();
    if (!enabled) {
      isClimbing = false;
    }

    /* TODO: suggestion from Leo: to prevent the climber from digging
      into ground when the cage is not completely in, we could check
      whether we have load at a specific angle that we expect to contact
      the cage it it was properly aligned. If not, we stop climb.
    */
    if (enabled && !atGoalAngle()) {
      double motorPower = isClimbing ? CLIMB_POWER.getValue() : NO_LOAD_POWER.getValue();
      motorPower *= currentAngle < goalAngle ? 1 : -1;

      mainMotor.set(motorPower);
      logMotorPower.append(motorPower);
    } else {
      mainMotor.set(0);
      logMotorPower.append(0);
    }
  }

  /** Sets the goal angle in radians and enables periodic control. */
  public void setGoalAngle(double angle) {
    angle = MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
    goalAngle = angle;
    enabled = true;

    logGoalAngle.append(angle);
    logEnabled.update(enabled);
  }

  /** Sets the goal angle to preprogrammed angle and supply climbing power to motors. */
  public void climb() {
    setGoalAngle(Math.toRadians(CLIMB_GOAL_ANGLE_DEG.getValue()));
    isClimbing = true;
  }

  public void disable() {
    enabled = false;
    logEnabled.update(enabled);
  }

  public boolean atGoalAngle() {
    return Math.abs(currentAngle - goalAngle) < Math.toRadians(TOLERANCE_DEG.getValue());
  }

  private void updateSensorState() {
    currentAngle = MathUtil.angleModulus(absoluteEncoder.get()); // in rad

    logCurrentAbsoluteAngle.append(currentAngle);
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
    GenericEntry angle = controlLayout.add("Angle", 0).getEntry();
    controlLayout.add(
        Commands.sequence(
                Commands.runOnce(() -> setGoalAngle(Math.toRadians(angle.getDouble(0))), this),
                Commands.idle(this).until(this::atGoalAngle))
            .withName("Set Angle (deg)"));
    controlLayout.add(Commands.runOnce(this::disable, this).withName("Disable"));
  }
}
