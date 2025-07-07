// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystems. */

  private PWMVictorSPX leftFront = new PWMVictorSPX(0);
  private PWMVictorSPX leftRear = new PWMVictorSPX(1);
  private PWMVictorSPX rightFront = new PWMVictorSPX(2);
  private PWMVictorSPX rightRear = new PWMVictorSPX(3);


  public TankDriveSubsystem() {

    rightFront.setInverted(true);
    rightRear.setInverted(true);
    

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftFront.set(leftSpeed);
    leftRear.set(leftSpeed);
    rightFront.set(rightSpeed);
    rightRear.set(rightSpeed);
  }

  public void stop() {
    tankDrive(0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
