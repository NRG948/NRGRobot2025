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

  private final double MIN_ANGLE = Math.toRadians(-100);
  private final double MAX_ANGLE = Math.toRadians(135); // TODO: verify real max angle
  private final double GEAR_RATIO = 5.0 * 5.0 * 66.0 / 18.0;
  // 360 - needed because abs encoder inversion is applied before offset
  private final double ABSOLUTE_ENCODER_ZERO_OFFSET = Math.toRadians(360 - 173.1); 

  @RobotPreferencesValue
  public static DoubleValue CLIMB_POWER = new DoubleValue("Climber", "Motor Power", 0.4);

  @RobotPreferencesValue
  public static DoubleValue TOLERANCE_DEG = new DoubleValue("Climber", "Tolerance (deg)", 1);

  @RobotPreferencesValue
  public static DoubleValue CLIMB_GOAL_ANGLE_DEG =
      new DoubleValue("Climber", "Climb Goal Angle (deg)", -88);
  
  @RobotPreferencesValue
  public static DoubleValue PROPORTIONAL_CONTROL_THRESHOLD_DEG = new DoubleValue("Climber", "Proportional Control Threshold", 10);

  private double currentAngle;
  private double goalAngle; // in radians
  private boolean enabled;

  private final DutyCycleEncoder absoluteEncoder;

  private TalonFXAdapter mainMotor =
      new TalonFXAdapter(
          new TalonFX(RobotConstants.CAN.TalonFX.CLIMBER_MAIN_MOTOR_ID, "rio"),
          false,
          true,
          1); // filler value; motor encoder value is not used.

  @SuppressWarnings("unused")
  private MotorController follower =
      mainMotor.createFollower(RobotConstants.CAN.TalonFX.CLIMBER_FOLLOWER_MOTOR_ID, false);

  private DoubleLogEntry logCurrentAbsoluteAngle =
      new DoubleLogEntry(LOG, "Climber/Absolute Angle");
  private DoubleLogEntry logGoalAngle = new DoubleLogEntry(LOG, "Climber/Goal Angle");
  private BooleanLogEntry logEnabled = new BooleanLogEntry(LOG, "Climber/Is Enabled");
  private DoubleLogEntry logMotorPower = new DoubleLogEntry(LOG, "Climber/Motor Power");
  private DoubleLogEntry logMotorStatorCurent = new DoubleLogEntry(LOG, "Climber/Motor Stator Current");
  private DoubleLogEntry logMotorTorqueCurent = new DoubleLogEntry(LOG, "Climber/Motor Torque Current");
  private DoubleLogEntry logMotorAngularVelocity = new DoubleLogEntry(LOG, "Climber/Motor Rot/s");

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

    /* TODO: suggestion from Leo: to prevent the climber from digging
      into ground when the cage is not completely in, we could check
      whether we have load at a specific angle that we expect to contact
      the cage it it was properly aligned. If not, we stop climb.
    */
    double angleError = goalAngle - currentAngle;
    double motorPower;
    if (enabled && !atGoalAngle()) {
      // Runs at climb power until within small angle of goal and then ramps power down linearly.
      double kP = CLIMB_POWER.getValue() / Math.toRadians(PROPORTIONAL_CONTROL_THRESHOLD_DEG.getValue());
      motorPower = MathUtil.clamp(kP * angleError, -CLIMB_POWER.getValue(), CLIMB_POWER.getValue());
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

  /** Sets the goal angle to preprogrammed angle and supply climbing power to motors. */
  public void climb() {
    setGoalAngle(Math.toRadians(CLIMB_GOAL_ANGLE_DEG.getValue()));
  }

  public void disable() {
    enabled = false;
    logEnabled.update(enabled);
  }

  public boolean atGoalAngle() {
    return MathUtil.isNear(goalAngle, currentAngle, Math.toRadians(TOLERANCE_DEG.getValue()));
  }

  private void updateSensorState() {
    currentAngle = MathUtil.angleModulus(absoluteEncoder.get()); // in rad

    logCurrentAbsoluteAngle.append(currentAngle);
    logMotorAngularVelocity.append(mainMotor.getEncoder().getVelocity()); // we set meter/rot = 1
    logMotorStatorCurent.append(mainMotor.getStatorCurrent());
    logMotorTorqueCurent.append(mainMotor.getTorqueCurrent());
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
