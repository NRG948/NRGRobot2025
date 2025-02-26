/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import static frc.robot.Constants.RobotConstants.CAN.TalonFX.ALGAE_GRABBER_MOTOR_ID;
import static frc.robot.util.MotorDirection.COUNTER_CLOCKWISE_POSITIVE;
import static frc.robot.util.MotorIdleMode.BRAKE;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MotorIdleMode;
import frc.robot.util.TalonFXAdapter;

public class AlgaeGrabber extends SubsystemBase implements ActiveSubsystem, ShuffleboardProducer {

  private final TalonFXAdapter motor =
      new TalonFXAdapter(
          new TalonFX(ALGAE_GRABBER_MOTOR_ID), COUNTER_CLOCKWISE_POSITIVE, BRAKE, 1.0);
  private double motorSpeed = 0;
  private boolean enabled;
  private boolean hasAlgae;

  /** Creates a new AlgaeGrabber. */
  public AlgaeGrabber() {}

  private void setMotorSpeed(double speed) {
    enabled = true;
    motorSpeed = speed;
  }

  public void intake() {
    // TODO: test & determine safe maximum speed
    setMotorSpeed(0.8);
  }

  public void hold() {
    // TODO: test & determine motor power needed to hang on to game piece while moving
    setMotorSpeed(0.2);
  }

  public void outtake() {
    setMotorSpeed(-0.8);
  }

  public boolean hasAlgae() {
    return hasAlgae;
  }

  @Override
  public void disable() {
    enabled = false;
    motor.stopMotor();
  }

  @Override
  public void setIdleMode(MotorIdleMode idleMode) {
    motor.setIdleMode(idleMode);
  }

  @Override
  public void periodic() {
    if (enabled) {
      motor.set(motorSpeed);
    }
  }

  public void addShuffleboardTab() {
    // if (!ENABLE_TAB.getValue()) {
    //   return;
    // }

    ShuffleboardTab rollerTab = Shuffleboard.getTab("Algae Grabber");

    ShuffleboardLayout statusLayout =
        rollerTab.getLayout("Status", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    statusLayout.addBoolean("Enabled", () -> enabled);

    ShuffleboardLayout controlLayout =
        rollerTab.getLayout("Control", BuiltInLayouts.kList).withPosition(2, 0).withSize(2, 4);
    GenericEntry speed = controlLayout.add("Speed", 0).getEntry();
    controlLayout.add(
        Commands.runOnce(() -> setMotorSpeed(speed.getDouble(0)), this).withName("Set Speed"));
    controlLayout.add(Commands.runOnce(this::disable, this).withName("Disable"));
  }
}
