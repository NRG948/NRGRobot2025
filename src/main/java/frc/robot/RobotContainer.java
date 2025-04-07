/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import static frc.robot.commands.AlgaeCommands.removeAlgaeAtLevel;
import static frc.robot.commands.CoralAndElevatorCommands.raiseElevatorAndTipCoralArm;
import static frc.robot.commands.CoralCommands.CORAL_GRABBER_DETECTION_DELAY;
import static frc.robot.commands.CoralCommands.CORAL_ROLLER_DETECTION_DELAY;
import static frc.robot.commands.DriveCommands.alignToReefPosition;
import static frc.robot.commands.DriveCommands.resetOrientation;
import static frc.robot.parameters.ElevatorLevel.AlgaeL2;
import static frc.robot.parameters.ElevatorLevel.AlgaeL3;
import static frc.robot.parameters.ElevatorLevel.L1;
import static frc.robot.parameters.ElevatorLevel.L2;
import static frc.robot.parameters.ElevatorLevel.L3;
import static frc.robot.parameters.ElevatorLevel.L4;
import static frc.robot.util.ReefPosition.CENTER_REEF;
import static frc.robot.util.ReefPosition.LEFT_BRANCH;
import static frc.robot.util.ReefPosition.RIGHT_BRANCH;

import au.grapplerobotics.CanBridge;
import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferences.EnumValue;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.CoralAndElevatorCommands;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.CoralRollerWithController;
import frc.robot.commands.DriveUsingController;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.FlameCycle;
import frc.robot.commands.LEDCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.parameters.Colors;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.MotorIdleMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Preferences", column = 0, row = 0, width = 2, height = 2)
public class RobotContainer {
  private static final int COAST_MODE_DELAY = 30;

  private static final DataLog LOG = DataLogManager.getLog();

  // The robot's subsystems and commands are defined here...
  private final Subsystems subsystems = new Subsystems();
  private final RobotAutonomous autonomous = new RobotAutonomous(subsystems, null);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController m_manipulatorController =
      new CommandXboxController(OperatorConstants.MANIPULATOR_CONTROLLER_PORT);

  private final Timer coastModeTimer = new Timer();
  private final StringLogEntry phaseLogger = new StringLogEntry(LOG, "/Robot/Phase");

  public enum RobotSelector {
    PracticeRobot2025,
    CompetitionRobot2025;
  }

  @RobotPreferencesValue
  public static EnumValue<RobotSelector> ROBOT_TYPE =
      new EnumValue<RobotSelector>("Preferences", "Robot Type", RobotSelector.CompetitionRobot2025);

  /** The container for the robot. Contains subsystems, OI devices, and command bindings. */
  public RobotContainer() {
    initShuffleboard();
    CanBridge.runTCP();

    subsystems.drivetrain.setDefaultCommand(
        new DriveUsingController(subsystems, m_driverController));

    subsystems.coralRoller.setDefaultCommand(
        new CoralRollerWithController(subsystems, m_manipulatorController));

    subsystems.statusLEDs.setDefaultCommand(new FlameCycle(subsystems.statusLEDs));

    // Configure the trigger bindings
    configureBindings();
  }

  public void disabledInit() {
    phaseLogger.append("Disabled");
    subsystems.disableAll();
    coastModeTimer.restart();
  }

  public void disabledPeriodic() {
    if (coastModeTimer.hasElapsed(COAST_MODE_DELAY)) {
      subsystems.setIdleMode(MotorIdleMode.COAST);
      coastModeTimer.stop();
      coastModeTimer.reset();
    }
  }

  public void autonomousInit() {
    phaseLogger.append("Autonomous");
    subsystems.setIdleMode(MotorIdleMode.BRAKE);
    subsystems.setInitialStates();
  }

  public void teleopInit() {
    phaseLogger.append("Teleop");
    subsystems.setIdleMode(MotorIdleMode.BRAKE);
    subsystems.setInitialStates();
  }

  private void initShuffleboard() {
    RobotPreferences.addShuffleBoardTab();

    subsystems.initShuffleboard();

    ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");
    autonomous.addShuffleboardLayout(operatorTab);

    operatorTab
        .addBoolean("Has Coral", () -> subsystems.coralRoller.hasCoral())
        .withSize(2, 2)
        .withPosition(6, 0);

    operatorTab
        .addBoolean("Branch Detected", () -> subsystems.coralRoller.detectsReef())
        .withSize(2, 2)
        .withPosition(6, 2);

    if (subsystems.frontCamera.isPresent()) {
      VideoSource video =
          new HttpCamera(
              "photonvision_Port_1184_Output_MJPEG_Server",
              "http://photonvision.local:1184/stream.mjpg",
              HttpCameraKind.kMJPGStreamer);

      operatorTab
          .add("Front Camera", video)
          .withWidget(BuiltInWidgets.kCameraStream)
          .withSize(4, 3)
          .withPosition(2, 0);
      operatorTab
          .add(
              "Reset QuestNav Pose",
              Commands.runOnce(
                      () -> {
                        Pose2d currentPose = subsystems.drivetrain.getPosition();
                        subsystems.questNav.setInitialRobotPose(currentPose);
                      })
                  .withName("Reset QuestNav Pose")
                  .ignoringDisable(true))
          .withSize(2, 1)
          .withPosition(4, 6);
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.start().onTrue(resetOrientation(subsystems));
    m_driverController.x().whileTrue(alignToReefPosition(subsystems, LEFT_BRANCH));
    m_driverController.y().whileTrue(alignToReefPosition(subsystems, CENTER_REEF));
    m_driverController.b().whileTrue(alignToReefPosition(subsystems, RIGHT_BRANCH));
    m_driverController.a().onTrue(ClimberCommands.prepareToClimb(subsystems));

    new Trigger(
            () -> {
              XboxController hid = m_driverController.getHID();
              return hid.getLeftBumperButton() && hid.getRightBumperButton();
            })
        .whileTrue(ClimberCommands.climb(subsystems));

    // Prepare to climb if any POV button is pressed
    new Trigger(() -> m_driverController.getHID().getPOV() != -1)
        .onTrue(ClimberCommands.prepareToClimb(subsystems));

    m_manipulatorController.a().onTrue(CoralCommands.extendToReefL1(subsystems));
    m_manipulatorController.x().onTrue(raiseElevatorAndTipCoralArm(subsystems, L2));
    m_manipulatorController.b().onTrue(raiseElevatorAndTipCoralArm(subsystems, L3));
    m_manipulatorController.y().onTrue(raiseElevatorAndTipCoralArm(subsystems, L4));

    m_manipulatorController.leftBumper().whileTrue(CoralCommands.manualGroundOuttake(subsystems));
    m_manipulatorController.leftBumper().onFalse(CoralAndElevatorCommands.stowAll(subsystems));
    m_manipulatorController.rightBumper().whileTrue(CoralCommands.intakeFromGround(subsystems));
    m_manipulatorController.rightBumper().onFalse(CoralCommands.extendToReefL1(subsystems));
    m_manipulatorController.povLeft().whileTrue(CoralCommands.intakeUntilCoralDetected(subsystems));
    m_manipulatorController.povRight().whileTrue(CoralCommands.scoreToReef(subsystems));
    m_manipulatorController.povRight().onFalse(CoralCommands.stowAfterScoring(subsystems));
    m_manipulatorController.povDown().whileTrue(removeAlgaeAtLevel(subsystems, AlgaeL2));
    m_manipulatorController.povUp().whileTrue(removeAlgaeAtLevel(subsystems, AlgaeL3));
    m_manipulatorController.povDown().onFalse(ElevatorCommands.stowElevatorAndArm(subsystems));
    m_manipulatorController.povUp().onFalse(ElevatorCommands.stowElevatorAndArm(subsystems));
    m_manipulatorController.back().onTrue(ManipulatorCommands.interruptAll(subsystems));
    m_manipulatorController.start().onTrue(CoralAndElevatorCommands.stowAll(subsystems));
    m_manipulatorController.rightTrigger().whileTrue(CoralCommands.autoCenterCoral(subsystems));
    m_manipulatorController.leftTrigger().whileTrue(CoralCommands.manualGroundOuttake(subsystems));
    m_driverController.leftTrigger().onFalse(CoralCommands.disableManualGroundOuttake(subsystems));

    new Trigger(subsystems.coralRoller::hasCoral)
        .onTrue(LEDCommands.indicateCoralAcquired(subsystems, CORAL_ROLLER_DETECTION_DELAY));
    new Trigger(subsystems.coralIntakeGrabber::hasCoral)
        .onTrue(LEDCommands.indicateCoralAcquired(subsystems, CORAL_GRABBER_DETECTION_DELAY));

    new Trigger(
            () ->
                subsystems.coralRoller.detectsReef() && subsystems.elevator.isSeekingAboveLevel(L1))
        .whileTrue(LEDCommands.indicateBranchDetected(subsystems));

    new Trigger(
            () ->
                subsystems.coralRoller.isLeftReefDetected()
                    && !subsystems.coralRoller.isRightReefDetected()
                    && subsystems.elevator.isSeekingAboveLevel(L1))
        .whileTrue(LEDCommands.setColor(subsystems.statusLEDs, Colors.BLUE));

    new Trigger(
            () ->
                !subsystems.coralRoller.isLeftReefDetected()
                    && subsystems.coralRoller.isRightReefDetected()
                    && subsystems.elevator.isSeekingAboveLevel(L1))
        .whileTrue(LEDCommands.setColor(subsystems.statusLEDs, Colors.ORANGE));

    new Trigger(
            () ->
                (!subsystems.coralRoller.isLeftReefDetected()
                        && !subsystems.coralRoller.isRightReefDetected())
                    || !subsystems.elevator.isSeekingAboveLevel(L1))
        .whileTrue(LEDCommands.setColor(subsystems.statusLEDs, Colors.BLACK));

    new Trigger(subsystems.coralArm::hasError)
        .whileTrue(LEDCommands.indicateErrorWithBlink(subsystems));
    new Trigger(subsystems.elevator::hasError)
        .whileTrue(LEDCommands.indicateErrorWithBlink(subsystems));
    new Trigger(subsystems.coralRoller::hasError)
        .whileTrue(LEDCommands.indicateErrorWithSolid(subsystems));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonomous.getAutonomousCommand(subsystems);
  }

  public void periodic() {
    subsystems.periodic();
  }
}
