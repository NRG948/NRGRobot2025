/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

@RobotPreferencesLayout(
    groupName = "AprilTag",
    column = 2,
    row = 2,
    width = 2,
    height = 2,
    type = "Grid Layout",
    gridColumns = 2,
    gridRows = 2)
public class AprilTagSubsystem extends SubsystemBase implements ShuffleboardProducer {
  /** Creates a new PhotonVisionSubsystem. */
  protected final PhotonCamera camera;

  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private PhotonPipelineResult result = new PhotonPipelineResult();
  private final PhotonPoseEstimator estimator;

  private double angleToTarget;
  private double distanceToTarget;
  private double poseAmibiguity;
  private int selectedAprilTag;

  @RobotPreferencesValue(column = 0, row = 0)
  public static final RobotPreferences.BooleanValue ENABLED =
      new RobotPreferences.BooleanValue("AprilTag", "Enabled", true);

  @RobotPreferencesValue(column = 1, row = 0)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("AprilTag", "Enable Tab", true);

  private final SendableChooser<Integer> aprilTagIdChooser = new SendableChooser<>();
  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo); // update to k2025Reefscape

  private BooleanLogEntry hasTargetLogger;
  private DoubleLogEntry distanceLogger;
  private DoubleLogEntry angleLogger;

  public AprilTagSubsystem(String cameraName, Transform3d robotToCamera) {
    this.camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
    this.cameraToRobot = robotToCamera.inverse();

    estimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

    for (int i = 1; i <= 22; i++) {
      aprilTagIdChooser.addOption(String.valueOf(i), i);
    }
    aprilTagIdChooser.setDefaultOption("1", 1);

    hasTargetLogger =
        new BooleanLogEntry(DataLogManager.getLog(), String.format("/%s/Has Target", cameraName));
    distanceLogger =
        new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Distance", cameraName));
    angleLogger =
        new DoubleLogEntry(DataLogManager.getLog(), String.format("/%s/Angle", cameraName));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    PhotonPipelineResult currentResult = camera.getLatestResult();
    if (this.result.hasTargets() != currentResult.hasTargets()) {
      estimator.update(currentResult);
      hasTargetLogger.append(currentResult.hasTargets());
    }

    if (ENABLE_TAB.getValue()) {
      selectedAprilTag = aprilTagIdChooser.getSelected().intValue();
    }

    this.result = currentResult;

    if (hasTargets()) {
      PhotonTrackedTarget bestTarget = getBestTarget();
      angleToTarget = -bestTarget.getYaw();
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
  protected PhotonPipelineResult getLatestResult() {
    return result;
  }

  /**
   * Return angle to specificed target
   *
   * @param target Target
   * @return Angle to target
   */
  public static double calculateAngleToTarget(PhotonTrackedTarget target) {
    return Math.toRadians(-target.getYaw());
  }

  /**
   * Return the transform from the camera to the center of the robot
   *
   * @return Return the transform from the camera to the center of the robot
   */
  public Transform3d getCameraToRobotTransform() {
    return cameraToRobot;
  }

  /***
   * Return the transform from the center of the robot to camera
   *
   * @return Return the transform from center of the robot to camera
   */
  public Transform3d getRobotToCameraTransform() {
    return robotToCamera;
  }

  /***
   * Returns weather the result has targets
   *
   * @return Boolean if result has targets
   */
  public boolean hasTargets() {
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

  /**
   * Returns the AprilTag target of the input ID.
   *
   * @param id The AprilTag ID.
   * @return The target with the input ID.
   */
  public Optional<PhotonTrackedTarget> getTarget(int id) {
    return getTargets().stream().filter(target -> target.getFiducialId() == id).findFirst();
  }

  /**
   * Returns the distance to the target with the input ID. Returns 0 if target not found.
   *
   * @param id The AprilTag ID.
   * @return The distance to the target with the input ID.
   */
  public double getDistanceToTarget(int id) {
    Optional<PhotonTrackedTarget> target = getTarget(id);
    if (target.isEmpty()) {
      return 0.0;
    }
    var bestCameraToTarget = target.get().getBestCameraToTarget();
    return Math.hypot(bestCameraToTarget.getX(), bestCameraToTarget.getY());
  }

  /**
   * Returns the angle to the target with the input ID. Returns 0 if target not found.
   *
   * @param id The AprilTag ID.
   * @return The angle to the target with the input ID.
   */
  public double getAngleToTarget(int id) {
    Optional<PhotonTrackedTarget> target = getTarget(id);
    if (target.isEmpty()) {
      return 0.0;
    }
    return calculateAngleToTarget(target.get());
  }

  public void addShuffleboardTab() {
    System.out.println("Running April Tag Shuffleboard");
    if (!ENABLE_TAB.getValue()) {
      return;
    }
    System.out.println("Added tab");
    VideoSource video =
        new HttpCamera(
            "photonvision_Port_1182_Output_MJPEG_Server",
            "http://photonvision.local:1190/stream.mjpg",
            HttpCameraKind.kMJPGStreamer);

    ShuffleboardTab visionTab = Shuffleboard.getTab("April Tag");
    ShuffleboardLayout targetLayout =
        visionTab.getLayout("Target Info", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5);
    targetLayout.add("ID Selection", aprilTagIdChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    targetLayout.addBoolean("Has Target", this::hasTargets);
    targetLayout.addDouble("Distance", () -> getDistanceToTarget(selectedAprilTag));
    targetLayout.addDouble("Angle", () -> Math.toDegrees(getAngleToTarget(selectedAprilTag)));

    visionTab
        .add("April Tag", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);

    ShuffleboardLayout aprilTagLayout =
        visionTab
            .getLayout("Target Position", BuiltInLayouts.kList)
            .withPosition(6, 0)
            .withSize(2, 4);
    ShuffleboardLayout selectedLayout =
        aprilTagLayout
            .getLayout("Selected April Tag", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of Columns", 3, "Number of Rows", 2));
  }
}
