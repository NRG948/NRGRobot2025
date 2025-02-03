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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.VisionParameters;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
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
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);
  public static final PhotonPipelineResult NO_RESULT = new PhotonPipelineResult();
  public static final Pose3d NO_APRILTAG = new Pose3d();
  public static final EstimatedRobotPose NO_APRILTAG_ESTIMATE =
      new EstimatedRobotPose(NO_APRILTAG, 0, List.of(), PoseStrategy.LOWEST_AMBIGUITY);
  public static final double LAST_RESULT_TIMEOUT = 0.1;

  @RobotPreferencesValue
  public static RobotPreferences.EnumValue<VisionParameters> PARAMETERS =
      new RobotPreferences.EnumValue<VisionParameters>(
          "AprilTag", "Robot Vision", VisionParameters.PracticeBase2025);

  @RobotPreferencesValue(column = 0, row = 0)
  public static final RobotPreferences.BooleanValue ENABLED =
      new RobotPreferences.BooleanValue("AprilTag", "Enabled", true);

  @RobotPreferencesValue(column = 1, row = 0)
  public static final RobotPreferences.BooleanValue ENABLE_TAB =
      new RobotPreferences.BooleanValue("AprilTag", "Enable Tab", true);

  private final PhotonCamera camera;
  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private final PhotonPoseEstimator estimator;

  private final SendableChooser<Integer> aprilTagIdChooser = new SendableChooser<>();
  private final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  private final BooleanLogEntry hasTargetLogger;
  private final DoubleLogEntry distanceLogger;
  private final DoubleLogEntry angleLogger;

  private Optional<PhotonPipelineResult> result = Optional.empty();
  private double angleToBestTarget;
  private double distanceToBestTarget;
  private double angleToSelectedTarget;
  private double distanceToSelectedTarget;

  private double poseAmibiguity;
  private int selectedAprilTag;
  private Pose3d selectedAprilTagPose = new Pose3d();
  private Optional<EstimatedRobotPose> globalEstimatedPose = Optional.empty();
  private Matrix<N3, N1> curStdDevs = SINGLE_TAG_STD_DEVS;

  /**
   * Constructs a new AprilTagSubsystem instance.
   *
   * @param cameraName The name of the camera.
   * @param robotToCamera The transform from the robot to the camera.
   */
  public AprilTagSubsystem(String cameraName, Transform3d robotToCamera) {
    setName(cameraName);
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

  /**
   * The latest estimated robot pose on the field from vision data. This may be empty. This should
   * only be called once per loop.
   *
   * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
   * {@link getEstimationStdDevs}
   *
   * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
   *     used for estimation.
   */
  public Optional<EstimatedRobotPose> getEstimateGlobalPose() {
    return this.globalEstimatedPose;
  }

  /**
   * Calculates new standard deviations for pose estimation.
   *
   * <p>This algorithm is a heuristic that creates dynamic standard deviations based on number of
   * tags, estimation strategy, and distance from the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      curStdDevs = SINGLE_TAG_STD_DEVS;
    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = SINGLE_TAG_STD_DEVS;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an average-distance metric
      for (var tgt : targets) {
        var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty()) {
          continue;
        }
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        curStdDevs = SINGLE_TAG_STD_DEVS;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple tags are visible
        if (numTags > 1) {
          estStdDevs = MULTI_TAG_STD_DEVS;
        }
        // Increase std devs based on (average) distance.
        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * The standard deviations of the estimated pose from {@link #getEstimateGlobalPose()}, for use
   * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   * This should only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return curStdDevs;
  }

  /**
   * Return the pose of the specified AprilTag.
   *
   * @param id The ID of the AprilTag.
   * @return The pose of the AprilTag.
   */
  public Pose3d getAprilTagPose(int id) {
    return aprilTagFieldLayout.getTagPose(id).orElse(NO_APRILTAG);
  }

  @Override
  public void periodic() {
    // Process the latest vision results updating the estimated robot pose and
    // current result.
    Optional<EstimatedRobotPose> visionEst = Optional.empty();
    Optional<PhotonPipelineResult> currentResult = Optional.empty();
    List<PhotonPipelineResult> allUnreadResults = camera.getAllUnreadResults();
    for (var change : allUnreadResults) {
      visionEst = estimator.update(change);
      updateEstimationStdDevs(visionEst, change.getTargets());
      currentResult = Optional.of(change);
    }

    globalEstimatedPose = visionEst;

    boolean hasTargets = currentResult.orElse(NO_RESULT).hasTargets();

    if (this.result.orElse(NO_RESULT).hasTargets() != hasTargets) {
      hasTargetLogger.append(hasTargets);
    }

    if (currentResult.isPresent()
        || (Timer.getFPGATimestamp() - this.result.orElse(NO_RESULT).getTimestampSeconds())
            > LAST_RESULT_TIMEOUT) {
      this.result = currentResult;
    }

    if (hasTargets()) {
      PhotonTrackedTarget bestTarget = getBestTarget();
      Transform3d bestTargetTransform = robotToCamera.plus(bestTarget.getBestCameraToTarget());
      distanceToBestTarget = Math.hypot(bestTargetTransform.getX(), bestTargetTransform.getY());
      angleToBestTarget = Math.atan2(bestTargetTransform.getY(), bestTargetTransform.getX());
      poseAmibiguity = bestTarget.getPoseAmbiguity();

      distanceLogger.append(distanceToBestTarget);
      angleLogger.append(angleToBestTarget);
    }

    if (ENABLE_TAB.getValue()) {
      selectedAprilTag = aprilTagIdChooser.getSelected().intValue();
      selectedAprilTagPose = getAprilTagPose(selectedAprilTag);

      Optional<PhotonTrackedTarget> target = getTarget(selectedAprilTag);
      if (target.isPresent()) {
        var robotToTarget = robotToCamera.plus(target.get().getBestCameraToTarget());
        distanceToSelectedTarget = Math.hypot(robotToTarget.getX(), robotToTarget.getY());
        angleToSelectedTarget = Math.atan2(robotToTarget.getY(), robotToTarget.getX());
      }
    }
  }

  /**
   * Returns the latest vision result
   *
   * @return latest vision result
   */
  protected PhotonPipelineResult getLatestResult() {
    return result.orElse(NO_RESULT);
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

  /** Returns whether one or more AprilTags are visible. */
  public boolean hasTargets() {
    return result.orElse(NO_RESULT).hasTargets();
  }

  /** Returns the best target AprilTag. */
  public PhotonTrackedTarget getBestTarget() {
    return result.orElse(NO_RESULT).getBestTarget();
  }

  /** Returns the distance in meters to the best target from the robot center. */
  public double getDistanceToBestTarget() {
    return distanceToBestTarget;
  }

  /** Returns angle in radians to best target relative to robot center. */
  public double getAngleToBestTarget() {
    return angleToBestTarget;
  }

  /** Returns the pose ambiguity of the best target. */
  public double getAmibiguity() {
    return poseAmibiguity;
  }

  /**
   * Returns the timestamp of the latest vision result. This is only valid if the subsystem has
   * targets.
   */
  public double getTargetTimeStamp() {
    return result.orElse(NO_RESULT).getTimestampSeconds();
  }

  /** Returns the visible AprilTag targets. */
  public List<PhotonTrackedTarget> getTargets() {
    return result.orElse(NO_RESULT).getTargets();
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
   * Returns the distance from center of the robot to the target with the input ID. Returns 0 if
   * target not found.
   *
   * @param id The AprilTag ID.
   * @return The distance to the target with the input ID.
   */
  public double getDistanceToTarget(int id) {
    Optional<PhotonTrackedTarget> target = getTarget(id);
    if (target.isEmpty()) {
      return 0.0;
    }
    var bestCameraToTarget = robotToCamera.plus(target.get().getBestCameraToTarget());
    return Math.hypot(bestCameraToTarget.getX(), bestCameraToTarget.getY());
  }

  /** Adds the AprilTag tab to Shuffleboard if enabled. */
  public void addShuffleboardTab() {
    if (!ENABLE_TAB.getValue()) {
      return;
    }
    VideoSource video =
        new HttpCamera(
            "photonvision_Port_1190_Output_MJPEG_Server",
            "http://photonvision.local:1190/stream.mjpg",
            HttpCameraKind.kMJPGStreamer);

    ShuffleboardTab visionTab = Shuffleboard.getTab(getName());
    ShuffleboardLayout targetLayout =
        visionTab.getLayout("Target Info", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 5);
    targetLayout.add("ID Selection", aprilTagIdChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    targetLayout.addBoolean("Has Target", this::hasTargets).withWidget(BuiltInWidgets.kBooleanBox);
    targetLayout
        .addDouble("Distance", () -> distanceToSelectedTarget)
        .withWidget(BuiltInWidgets.kTextView);
    targetLayout
        .addDouble("Angle", () -> Math.toDegrees(angleToSelectedTarget))
        .withWidget(BuiltInWidgets.kTextView);

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
    selectedLayout.addDouble("X", () -> selectedAprilTagPose.getX()).withPosition(0, 0);
    selectedLayout.addDouble("Y", () -> selectedAprilTagPose.getY()).withPosition(1, 0);
    selectedLayout.addDouble("Z", () -> selectedAprilTagPose.getZ()).withPosition(2, 0);
    selectedLayout
        .addDouble("Roll", () -> Math.toDegrees(selectedAprilTagPose.getRotation().getX()))
        .withPosition(0, 1);
    selectedLayout
        .addDouble("Pitch", () -> Math.toDegrees(selectedAprilTagPose.getRotation().getY()))
        .withPosition(1, 1);
    selectedLayout
        .addDouble("Yaw", () -> Math.toDegrees(selectedAprilTagPose.getRotation().getZ()))
        .withPosition(2, 1);

    ShuffleboardLayout estimatedLayout =
        aprilTagLayout
            .getLayout("Global Estimated Pose", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 3, "Number of rows", 2));
    estimatedLayout
        .addDouble("X", () -> globalEstimatedPose.orElse(NO_APRILTAG_ESTIMATE).estimatedPose.getX())
        .withPosition(0, 0);
    estimatedLayout
        .addDouble("Y", () -> globalEstimatedPose.orElse(NO_APRILTAG_ESTIMATE).estimatedPose.getY())
        .withPosition(1, 0);
    estimatedLayout
        .addDouble("Z", () -> globalEstimatedPose.orElse(NO_APRILTAG_ESTIMATE).estimatedPose.getZ())
        .withPosition(2, 0);
    estimatedLayout
        .addDouble(
            "Roll",
            () ->
                Math.toDegrees(
                    globalEstimatedPose
                        .orElse(NO_APRILTAG_ESTIMATE)
                        .estimatedPose
                        .getRotation()
                        .getX()))
        .withPosition(0, 1);
    estimatedLayout
        .addDouble(
            "Pitch",
            () ->
                Math.toDegrees(
                    globalEstimatedPose
                        .orElse(NO_APRILTAG_ESTIMATE)
                        .estimatedPose
                        .getRotation()
                        .getY()))
        .withPosition(1, 1);
    estimatedLayout
        .addDouble(
            "Yaw",
            () ->
                Math.toDegrees(
                    globalEstimatedPose
                        .orElse(NO_APRILTAG_ESTIMATE)
                        .estimatedPose
                        .getRotation()
                        .getZ()))
        .withPosition(2, 1);
  }
}
