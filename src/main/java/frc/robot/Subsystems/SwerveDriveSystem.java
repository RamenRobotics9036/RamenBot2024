// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.DriveSwerveCommand;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.Constants.SwerveSystemConstants.SwerveSystemDeviceConstants;
import frc.robot.Util.AppliedController;

public class SwerveDriveSystem extends SubsystemBase {
  GenericEntry m_getPIDDriveP;
  GenericEntry m_getPIDDriveD;

  GenericEntry m_getPIDTurnP;
  GenericEntry m_getPIDTurnD;

  public static final boolean isPIDTuning = SwerveSystemConstants.isPIDTuning;

  private final double maxSpeed = SwerveSystemConstants.maxSpeedMetersPerSecond;
  private final double maxAngularSpeed = SwerveSystemConstants.maxAngularSpeed;

  private final Translation2d m_frontLeftLocation = new Translation2d(SwerveSystemConstants.frameDistanceToModulesMeters, SwerveSystemConstants.frameDistanceToModulesMeters);
  private final Translation2d m_frontRightLocation = new Translation2d(SwerveSystemConstants.frameDistanceToModulesMeters, -SwerveSystemConstants.frameDistanceToModulesMeters);
  private final Translation2d m_backLeftLocation = new Translation2d(-SwerveSystemConstants.frameDistanceToModulesMeters, SwerveSystemConstants.frameDistanceToModulesMeters);
  private final Translation2d m_backRightLocation = new Translation2d(-SwerveSystemConstants.frameDistanceToModulesMeters, -SwerveSystemConstants.frameDistanceToModulesMeters);

  private final SwerveModule m_frontLeft = new SwerveModule(
      SwerveSystemDeviceConstants.frontLeftDriveMotorID,
      SwerveSystemDeviceConstants.frontLeftTurnMotorID,
      SwerveSystemDeviceConstants.frontLeftTurnEncoderChannel,
      SwerveSystemDeviceConstants.frontLeftOffset
    );

    private final SwerveModule m_frontRight = new SwerveModule(
      SwerveSystemDeviceConstants.frontRightDriveMotorID,
      SwerveSystemDeviceConstants.frontRightTurnMotorID,
      SwerveSystemDeviceConstants.frontRightTurnEncoderChannel,
      SwerveSystemDeviceConstants.frontRightOffset
    );

  private final SwerveModule m_backLeft = new SwerveModule(
      SwerveSystemDeviceConstants.backLeftDriveMotorID,
      SwerveSystemDeviceConstants.backLeftTurnMotorID,
      SwerveSystemDeviceConstants.backLeftTurnEncoderChannel,
      SwerveSystemDeviceConstants.backLeftOffset
    );

    private final SwerveModule m_backRight = new SwerveModule(
      SwerveSystemDeviceConstants.backRightDriveMotorID,
      SwerveSystemDeviceConstants.backRightTurnMotorID,
      SwerveSystemDeviceConstants.backRightTurnEncoderChannel,
      SwerveSystemDeviceConstants.backRightOffset
    );

    private final Pigeon2 m_gyro = new Pigeon2(SwerveSystemConstants.gyroCanID);

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
      Rotation2d.fromDegrees(-getAnglePosition()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      });

  public SwerveDriveSystem(AppliedController controller) {
    initShuffleBoard();
    setDefaultCommand(
        new DriveSwerveCommand(this, controller)
    );
  }

  public void initShuffleBoard() {
    m_frontLeft.displayDesiredStateToDashBoard("Front Left");
    m_backLeft.displayDesiredStateToDashBoard("Back Left");
    m_frontRight.displayDesiredStateToDashBoard("Front Right");
    m_backRight.displayDesiredStateToDashBoard("Back Right");

    // Also display all Swerve values on a SINGLE dashboard using a Grid layout
    displayModuleToSingleSwerveDashV2("Front Left", m_frontLeft);
    displayModuleToSingleSwerveDashV2("Back Left", m_backLeft);
    displayModuleToSingleSwerveDashV2("Front Right", m_frontRight);
    displayModuleToSingleSwerveDashV2("Back Right", m_backRight);

    if (isPIDTuning) {
      m_getPIDDriveP = Shuffleboard.getTab("Swerve Tuning").getLayout("PID Tuning Drive Values", BuiltInLayouts.kList).add("Drive P", SwerveModule.pidDriveP).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
      m_getPIDDriveD = Shuffleboard.getTab("Swerve Tuning").getLayout("PID Tuning Drive Values", BuiltInLayouts.kList).add("Drive D", SwerveModule.pidDriveD).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

      m_getPIDTurnP = Shuffleboard.getTab("Swerve Tuning").getLayout("PID Tuning Turn Values", BuiltInLayouts.kList).add("Turn P", SwerveModule.pidTurnP).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
      m_getPIDTurnD = Shuffleboard.getTab("Swerve Tuning").getLayout("PID Tuning Turn Values", BuiltInLayouts.kList).add("Turn D", SwerveModule.pidTurnD).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    }
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    drive(xSpeed, ySpeed, rot, false);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot * maxAngularSpeed, makeRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot * maxAngularSpeed));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void displaySwerveStateToDashBoard(String name, SwerveModuleState state) {
    ShuffleboardTab tab = Shuffleboard.getTab(name);
    tab.add("Angle", state.angle);
    tab.add("Speed Meters", state.speedMetersPerSecond);
  }

  private double roundTo3Digits(double value) {
    return Math.round(value * 1000.0) / 1000.0;
  }

  private void addItemToGrid(ShuffleboardLayout grid, String name, DoubleSupplier valueSupplier, int row) {
    grid.addString("Label" + Integer.toString(row), () -> name)
        .withPosition(0, row);
    grid.addDouble(name, valueSupplier)
        .withPosition(1, row);
  }

  // Class to hold two integers, x,y
  private class IntPos {
    public int x;
    public int y;

    private IntPos(int x, int y) {
      this.x = x;
      this.y = y;
    }
  }

  private void displayModuleToSingleSwerveDashV2(String name, SwerveModule module) {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
    final Map<String, IntPos> gridPositions = Map.of(
        "Front Left", new IntPos(2, 2),
        "Front Right", new IntPos(6, 2),
        "Back Left", new IntPos(2, 4),
        "Back Right", new IntPos(6, 4));

    // Get the desired position of the grid widget, based on the name
    // of the module. If the name is not found, use 0,0.
    IntPos pos = gridPositions.get(name);
    if (pos == null) {
      pos = new IntPos(0, 0);
    }

    // Create a List widget to hold the formatted strings
    int numGridItems = 4;
    ShuffleboardLayout grid = tab.getLayout(name, BuiltInLayouts.kGrid)
        .withPosition(pos.x, pos.y)
        .withSize(4, 2)
        .withProperties(Map.of("Label position", "HIDDEN",
            "Number of columns", 2,
            "Number of rows", numGridItems));

    addItemToGrid(grid, "Turn Relative Encoder", () -> roundTo3Digits(module.getTurnEncoderRotations()), 0);
    addItemToGrid(grid, "Turn Absolute Encoder", () -> roundTo3Digits(module.getRawTurnEncoderValue()), 1);
    addItemToGrid(grid, "Drive Velocity", () -> roundTo3Digits(module.getDriveEncoderVelocity()), 2);
    // addItemToGrid(grid, "Unoptimized setpoint",
    // () -> roundTo2Digits(module.getUnoptimizedTurningSetpointRotations()), 3);
    // addItemToGrid(grid, "Turn setpoint", () ->
    // roundTo2Digits(module.getTurningSetpointRotations()), 4);
    addItemToGrid(grid, "Turn offset", () -> roundTo3Digits(module.getOffset()), 3);
  }

  public void updateOdometry() {
    m_odometry.update(
        makeRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
  }

  public double getXPosition() {
    return m_odometry.getPoseMeters().getX();
  }

  public double getYPosition() {
    return m_odometry.getPoseMeters().getX();
  }

  public double getAnglePosition() {
    return m_gyro.getRoll();
  }

  public Rotation2d makeRotation2d() {
    return Rotation2d.fromRotations(getAnglePosition());
  }

  public void updatePIDFromShuffleBoard() {
    if (isPIDTuning) {
      double pidDriveP = m_getPIDDriveP.getDouble(SwerveModule.pidDriveP);
      double pidDriveD = m_getPIDDriveD.getDouble(SwerveModule.pidDriveD);

      double pidTurnP = m_getPIDTurnP.getDouble(SwerveModule.pidTurnP);
      double pidTurnD = m_getPIDTurnD.getDouble(SwerveModule.pidTurnD);

      m_frontLeft.updateDrivePID(pidDriveP, pidDriveD);
      m_frontRight.updateDrivePID(pidDriveP, pidDriveD);
      m_backLeft.updateDrivePID(pidDriveP, pidDriveD);
      m_backRight.updateDrivePID(pidDriveP, pidDriveD);

      m_frontLeft.updateTurnPID(pidTurnP, pidTurnD);
      m_frontRight.updateTurnPID(pidTurnP, pidTurnD);
      m_backLeft.updateTurnPID(pidTurnP, pidTurnD);
      m_backRight.updateTurnPID(pidTurnP, pidTurnD);
    }
  }

  @Override
  public void periodic() {
    updatePIDFromShuffleBoard();
    updateOdometry();
  }

  public void stopSystem() {
    m_frontLeft.stopSystem();
    m_frontRight.stopSystem();
    m_backLeft.stopSystem();
    m_backRight.stopSystem();
  }
}