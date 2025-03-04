/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot;

import static frc.robot.commands.AlgaeCommands.removeAlgaeAtLevel;
import static frc.robot.commands.CoralAndElevatorCommands.raiseElevatorAndTipCoralArm;
import static frc.robot.commands.CoralCommands.outtakeUntilCoralNotDetected;
import static frc.robot.parameters.ElevatorLevel.AlgaeL2;
import static frc.robot.parameters.ElevatorLevel.AlgaeL3;
import static frc.robot.parameters.ElevatorLevel.L1;
import static frc.robot.parameters.ElevatorLevel.L2;
import static frc.robot.parameters.ElevatorLevel.L3;
import static frc.robot.parameters.ElevatorLevel.L4;
import static frc.robot.util.ReefPosition.CENTER_REEF;
import static frc.robot.util.ReefPosition.LEFT_BRANCH;
import static frc.robot.util.ReefPosition.RIGHT_BRANCH;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferences.EnumValue;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.CoralCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveUsingController;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.FlameCycle;
import frc.robot.commands.LEDCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.AprilTag;
import frc.robot.subsystems.AprilTag.VisionParameters;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberParameters;
import frc.robot.subsystems.Subsystems;
import frc.robot.util.MotorIdleMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@RobotPreferencesLayout(groupName = "Preferences", column = 0, row = 0, width = 1, height = 2)
public class RobotContainer {
  private static final int COAST_MODE_DELAY = 10;

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
    PracticeRobot2025(AprilTag.VISION_PARAMS, Climber.PRACTICE_BOT_PARAMETERS),
    CompetitionRobot2025(AprilTag.VISION_PARAMS, Climber.COMPETITION_BOT_PARAMETERS);

    private final VisionParameters visionParams;
    private final ClimberParameters climberParams;

    private RobotSelector(VisionParameters visionParams, ClimberParameters climberParams) {
      this.visionParams = visionParams;
      this.climberParams = climberParams;
    }

    public VisionParameters visionParameters() {
      return visionParams;
    }

    public ClimberParameters climberParameters() {
      return climberParams;
    }
  }

  @RobotPreferencesValue
  public static EnumValue<RobotSelector> PARAMETERS =
      new EnumValue<RobotSelector>("Preferences", "Robot Type", RobotSelector.CompetitionRobot2025);

  /** The container for the robot. Contains subsystems, OI devices, and command bindings. */
  public RobotContainer() {
    initShuffleboard();

    subsystems.drivetrain.setDefaultCommand(
        new DriveUsingController(subsystems, m_driverController));

    // subsystems.coralRoller.setDefaultCommand(
    //    new CoralRollerWithController(subsystems, m_manipulatorController));

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
  }

  public void teleopInit() {
    phaseLogger.append("Teleop");
    subsystems.setIdleMode(MotorIdleMode.BRAKE);
  }

  private void initShuffleboard() {
    RobotPreferences.addShuffleBoardTab();

    subsystems.initShuffleboard();

    ShuffleboardTab operatorTab = Shuffleboard.getTab("Operator");
    autonomous.addShuffleboardLayout(operatorTab);
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
    m_driverController.start().onTrue(DriveCommands.resetOrientation(subsystems));
    m_driverController.x().whileTrue(DriveCommands.alignToReefPosition(subsystems, LEFT_BRANCH));
    m_driverController.y().whileTrue(DriveCommands.alignToReefPosition(subsystems, CENTER_REEF));
    m_driverController.b().whileTrue(DriveCommands.alignToReefPosition(subsystems, RIGHT_BRANCH));
    m_driverController.a().whileTrue(DriveCommands.alignToCoralStation(subsystems));
    new Trigger(
            () -> {
              XboxController hid = m_driverController.getHID();
              return hid.getLeftBumperButton() && hid.getRightBumperButton();
            })
        .whileTrue(ClimberCommands.climb(subsystems));

    m_manipulatorController.a().onTrue(raiseElevatorAndTipCoralArm(subsystems, L1));
    m_manipulatorController.x().onTrue(raiseElevatorAndTipCoralArm(subsystems, L2));
    m_manipulatorController.b().onTrue(raiseElevatorAndTipCoralArm(subsystems, L3));
    m_manipulatorController.y().onTrue(raiseElevatorAndTipCoralArm(subsystems, L4));

    m_manipulatorController.rightBumper().whileTrue(AlgaeCommands.intakeAlgae(subsystems));
    m_manipulatorController.rightBumper().onFalse(AlgaeCommands.stopAndStowIntake(subsystems));
    m_manipulatorController.leftBumper().whileTrue(AlgaeCommands.outtakeAlgae(subsystems));
    m_manipulatorController.leftBumper().onFalse(AlgaeCommands.stopAndStowIntake(subsystems));
    m_manipulatorController.povLeft().whileTrue(CoralCommands.intakeUntilCoralDetected(subsystems));
    m_manipulatorController.povRight().whileTrue(outtakeUntilCoralNotDetected(subsystems));
    m_manipulatorController
        .povRight()
        .onFalse(ElevatorCommands.stowElevatorAndArmForCoral(subsystems));
    m_manipulatorController.povDown().whileTrue(removeAlgaeAtLevel(subsystems, AlgaeL2));
    m_manipulatorController.povUp().whileTrue(removeAlgaeAtLevel(subsystems, AlgaeL3));
    m_manipulatorController.povDown().onFalse(ElevatorCommands.stowElevatorAndArm(subsystems));
    m_manipulatorController.povUp().onFalse(ElevatorCommands.stowElevatorAndArm(subsystems));
    m_manipulatorController.back().onTrue(ManipulatorCommands.interruptAll(subsystems));
    m_manipulatorController.start().onTrue(ElevatorCommands.stowElevatorAndArm(subsystems));

    new Trigger(subsystems.coralRoller::hasCoral)
        .onTrue(LEDCommands.indicateCoralAcquired(subsystems));
    subsystems.algaeGrabber.ifPresent(
        (algaeGrabber) -> {
          new Trigger(algaeGrabber::hasAlgae).onTrue(LEDCommands.indicateAlgaeAcquired(subsystems));
        });

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
