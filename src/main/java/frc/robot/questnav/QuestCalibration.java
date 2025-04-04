/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;
import java.util.function.Supplier;

public class QuestCalibration {

  private static final DataLog LOG = DataLogManager.getLog();

  // -- Calculate Quest Offset (copied from
  // https://github.com/FRC5010/Reefscape2025/blob/main/TigerShark2025/src/main/java/org/frc5010/common/sensors/camera/QuestNav.java#L65) --

  private Translation2d calculatedOffsetToRobot = new Translation2d();
  private double calculateOffsetCount = 1;

  private Translation2d calculateOffsetToRobot(Pose2d robotPose) {
    Rotation2d angle = robotPose.getRotation();
    Translation2d displacement = robotPose.getTranslation();

    double x =
        ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
            / (2 * (1 - angle.getCos()));
    double y =
        ((-angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
            / (2 * (1 - angle.getCos()));

    return new Translation2d(x, y);
  }

  public Command determineOffsetToRobotCenter(
      Swerve drive, Supplier<Pose2d> robotPose, Supplier<Pose2d> questPoseSupplier) {
    return Commands.repeatingSequence(
            Commands.run(() -> drive.drive(0, 0, Math.PI / 10.0, false), drive)
                .withTimeout(0.5), // TODO: Pass in valid rotaional speed
            Commands.runOnce(
                    () -> {
                      // Update current offset
                      Translation2d offset = calculateOffsetToRobot(robotPose.get());

                      calculatedOffsetToRobot =
                          calculatedOffsetToRobot
                              .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                              .plus(offset.div(calculateOffsetCount + 1));
                      calculateOffsetCount++;
                      // Logger.recordOutput(
                      //     "QuestCalibration/CalculatedOffset", calculatedOffsetToRobot);
                    })
                .onlyIf(() -> questPoseSupplier.get().getRotation().getDegrees() > 30))
        .finallyDo(
            () -> {
              // Update current offset
              Translation2d offset = calculateOffsetToRobot(robotPose.get());

              calculatedOffsetToRobot =
                  calculatedOffsetToRobot
                      .times((double) calculateOffsetCount / (calculateOffsetCount + 1))
                      .plus(offset.div(calculateOffsetCount + 1));
              calculateOffsetCount++;
              // Logger.recordOutput("QuestCalibration/CalculatedOffset", calculatedOffsetToRobot);
            });
  }
}
