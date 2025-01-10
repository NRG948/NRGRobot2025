// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotonVisionSubsystem. */
  protected final PhotonCamera camera;

  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private PhotonPipelineResult result = new PhotonPipelineResult();
  
  private double angleToTarget;
  private double distanceToTarget;
  private double poseAmibiguity;

  private BooleanLogEntry hasTargetLogger;
  private DoubleLogEntry distanceLogger;
  private DoubleLogEntry angleLogger;

  public PhotonVisionSubsystem(String cameraName, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.cameraToRobot = robotToCamera.inverse();
    this.robotToCamera = robotToCamera;

    hasTargetLogger = new BooleanLogEntry(DataLogManager.getLog(), String.format("/%s/Has Target", cameraName));
    distanceLogger = new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Distance", cameraName));
    angleLogger = new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Angle", cameraName));

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult currentResult = camera.getLatestResult();
    if (this.result.hasTargets() != currentResult.hasTargets()) {
      hasTargetLogger.append(currentResult.hasTargets());
    }

    this.result = currentResult;

    if (hasTargets()) {
      PhotonTrackedTarget bestTarget = getBestTarget();
      angleToTarget =- bestTarget.getYaw();
      Transform3d bestTargetTransform = bestTarget.getBestCameraToTarget();
      distanceToTarget = Math.hypot(bestTargetTransform.getX(), bestTargetTransform.getY());
      poseAmibiguity = bestTarget.getPoseAmbiguity();

      distanceLogger.append(distanceToTarget);
      angleLogger.append(angleToTarget);
    }
  }

  /**
   * Returns the latest vision result
   * 
   * @return latest vision result
   */
  protected PhotonPipelineResult getLatestResult(){
    return result;
  }

  /**
   * Return angle to specificed target
   * @param target Target
   * @return Angle to target
   */
  public static double calculateAngletToTarget(PhotonTrackedTarget target){
    return Math.toRadians(-target.getYaw());
  }

  /**
   * Return the transform from the camera to the center of the robot 
   * 
   * @return Return the transform from the camera to the center of the robot
   */

  public Transform3d getCameraToRobotTransform(){
    return cameraToRobot;
  }

  /***
   * Return the transform from the center of the robot to camera
   * 
   * @return Return the transform from center of the robot to camera
   */
  public Transform3d getRobotToCameraTransform(){
    return robotToCamera;
  }

  /***
   * Returns weather the result has targets
   * 
   * @return Boolean if result has targets
   */
  public boolean hasTargets(){
    return result.hasTargets();
  }

  /***
   * Return best target
   * 
   * @return best target
   */
  public PhotonTrackedTarget getBestTarget() {
    return result.getBestTarget();
  }

  public double getDistanceToBestTarget() {
    return distanceToTarget;
  }

  public double getAngleToBestTarget() {
    return angleToTarget;
  }
  
  public double getAmibiguity() {
    return poseAmibiguity;
  }

  public double getTargetTimeStamp() {
    return result.getTimestampSeconds();
  }

  public List<PhotonTrackedTarget> getTargets() {
    return result.getTargets();
  }

}
