/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.parameters.SwerveDriveParameters;
import frc.robot.util.SwerveModuleVelocities;
import frc.robot.util.SwerveModuleVoltages;
import java.util.Optional;
import java.util.function.Supplier;

/** SwerveDrive implements swerve drive control. */
public class SwerveDrive extends RobotDriveBase {
  private static final ChassisSpeeds ZERO_SPEEDS = new ChassisSpeeds();

  private final SwerveModule[] modules;
  private final SwerveDriveKinematics kinematics;
  private final Supplier<Rotation2d> orientationSupplier;
  private final double maxDriveSpeed;
  private final double maxRotationalSpeed;

  // The current supplied state updated by the periodic method.
  private Rotation2d orientation;

  private DoubleLogEntry xSpeedLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/SwerveDrive/xSpeed");
  private DoubleLogEntry ySpeedLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/SwerveDrive/ySpeed");
  private DoubleLogEntry omegaSpeedLog =
      new DoubleLogEntry(DataLogManager.getLog(), "/SwerveDrive/omegaSpeed");

  private final SwerveModuleState[] moduleStates = new SwerveModuleState[4];
  private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
  private final SwerveModuleVelocities[] velocities = new SwerveModuleVelocities[4];
  private final SwerveModuleVoltages[] motorVoltages = new SwerveModuleVoltages[4];

  /**
   * constructs the swerve drive
   *
   * @param parameters A {@link SwerveDriveParameters} object providing information on the physical
   *     swerve drive characteristics.
   * @param modules An array of four {@link SwerveModule} objects in the order: front left, front
   *     right, back left, back right.
   * @param kinematics A {@link SwerveDriveKinematics} object used to convert chassis velocity into
   *     individual module states.
   * @param orientationSupplier Supplies the robot orientation relative to the field.
   */
  public SwerveDrive(
      SwerveDriveParameters parameters,
      SwerveModule[] modules,
      Supplier<Rotation2d> orientationSupplier) {
    this.modules = modules;
    this.kinematics = parameters.getKinematics();
    this.orientationSupplier = orientationSupplier;
    this.maxDriveSpeed = parameters.getMaxDriveSpeed();
    this.maxRotationalSpeed = parameters.getMaxRotationalSpeed();

    initializeSuppliedState();
  }

  /** Initializes the supplied state. */
  private void initializeSuppliedState() {
    updateSuppliedState();
  }

  /**
   * Updates the supplied state.
   *
   * <p>This method **MUST* be called by the {@link #periodic()} method to ensure the supplied state
   * is up to date for subsequent use.
   */
  private void updateSuppliedState() {
    orientation = orientationSupplier.get();
  }

  /**
   * Sets the swerve module states.
   *
   * @param states An array of four {@link SwerveModuleState} objects in the order: front left,
   *     front right, back left, back right
   */
  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, maxDriveSpeed);

    for (int i = 0; i < modules.length; ++i) {
      modules[i].setState(states[i]);
    }

    // Reset the motor watchdog timer.
    feedWatchdog();
  }

  @Override
  public void stopMotor() {
    setChassisSpeeds(ZERO_SPEEDS);

    for (SwerveModule module : modules) {
      module.stopMotors();
    }
  }

  @Override
  public String getDescription() {
    return "SwerveDrive";
  }

  /**
   * Returns the orientation of the robot frame relative to the field.
   *
   * @return The orientation of the robot frame.
   */
  public Rotation2d getOrientation() {
    return orientation;
  }

  /**
   * Drives the robot based on joystick inputs.
   *
   * @param xSpeed Speed of the robot in the x direction.
   * @param ySpeed Speed of the robot in the y direction.
   * @param rSpeed Rotation speed of the robot.
   * @param fieldRelative Whether the x and y values are relative to field.
   */
  public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative) {
    // multiples all values with max speed.
    xSpeed *= m_maxOutput * maxDriveSpeed;
    ySpeed *= m_maxOutput * maxDriveSpeed;
    rSpeed *= m_maxOutput * maxRotationalSpeed;

    if (!fieldRelative) {
      setChassisSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rSpeed));
    } else {
      Optional<Alliance> alliance = DriverStation.getAlliance();

      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        xSpeed *= -1.0;
        ySpeed *= -1.0;
      }

      setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, orientation));
    }
  }

  /**
   * Returns the current module state describing the wheel velocity and angle.
   *
   * @return The current module state.
   */
  public SwerveModuleState[] getModuleStates() {
    for (int i = 0; i < modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }

    return moduleStates;
  }

  /**
   * Sets the current module's states based on the chassis speed.
   *
   * @param speeds The chassis speeds.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    xSpeedLog.append(speeds.vxMetersPerSecond);
    ySpeedLog.append(speeds.vyMetersPerSecond);
    omegaSpeedLog.append(Math.toDegrees(speeds.omegaRadiansPerSecond));

    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

    setModuleStates(states);
  }

  /**
   * Returns the current chassis speed.
   *
   * @return The chassis speed.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Returns the swerve module positions.
   *
   * @return Swerve module positions.
   */
  public SwerveModulePosition[] getModulesPositions() {
    for (int i = 0; i < modules.length; i++) {
      modulePositions[i] = modules[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * Returns the swerve module velocities.
   *
   * @return The swerve module velocities.
   */
  public SwerveModuleVelocities[] getModuleVelocities() {
    for (int i = 0; i < modules.length; i++) {
      velocities[i] = modules[i].getVelocities();
    }

    return velocities;
  }

  /**
   * Gets the module motor voltages.
   *
   * @return The module motor
   */
  public SwerveModuleVoltages[] getModuleVoltages() {
    for (int i = 0; i < modules.length; i++) {
      motorVoltages[i] = modules[i].getMotorVoltages();
    }
    return motorVoltages;
  }

  /**
   * Sets the module motor voltages.
   *
   * @param moduleVoltages The module motor voltages.
   */
  public void setModuleVoltages(SwerveModuleVoltages[] moduleVoltages) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setMotorVoltages(moduleVoltages[i]);
    }

    feedWatchdog();
  }

  /**
   * This method is called periodically by the SwerveSubsystem. It is used to update drive-specific
   * state.
   */
  public void periodic() {
    updateSuppliedState();

    for (SwerveModule module : modules) {
      module.periodic();
    }
  }

  /**
   * This method is called periodically by the SwerveSubsystem. It is used to update module-specific
   * simulation state.
   */
  public void simulationPeriodic() {
    for (SwerveModule module : modules) {
      module.simulationPeriodic();
    }
  }

  /**
   * Adds the SwerveModule layouts to the shuffleboard tab.
   *
   * @param tab The suffleboard tab to add layouts
   */
  public void addShuffleboardLayouts(ShuffleboardTab tab) {
    for (int i = 0; i < modules.length; i++) {
      modules[i]
          .addShuffleboardLayout(tab)
          .withSize(3, 2)
          .withPosition((i * 3) % 6, ((i / 2) * 2) % 4);
    }
  }
}
