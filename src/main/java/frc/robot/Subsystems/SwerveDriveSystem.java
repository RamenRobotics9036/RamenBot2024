// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.DriveSwerveCommand;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.Constants.SwerveSystemConstants.SwerveSystemDeviceConstants;
import frc.robot.Util.AppliedController;

public class SwerveDriveSystem extends SubsystemBase {
  private final double maxSpeed = SwerveSystemConstants.maxSpeedMetersPerSecond;

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

    private final Pigeon2 m_gyro = new Pigeon2(SwerveSystemConstants.gyroChannelPWM);

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
    setDefaultCommand(
        new DriveSwerveCommand(this, controller)
    );

    m_frontLeft.displayDesiredStateToDashBoard("Front Left");
    m_backLeft.displayDesiredStateToDashBoard("Back Left");
    m_frontRight.displayDesiredStateToDashBoard("Front Right");
    m_backRight.displayDesiredStateToDashBoard("Back Right");

    displayModuleToDashBoard("Front Left", m_frontLeft);
    displayModuleToDashBoard("Front Right", m_frontRight);
    displayModuleToDashBoard("Back Left", m_backLeft);
    displayModuleToDashBoard("Back Right", m_backRight);
  }

  public void drive(double xSpeed, double ySpeed, double rot) {
    drive(xSpeed, ySpeed, rot, false);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, makeRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
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

  public void displayModuleToDashBoard(String name, SwerveModule module) {
    ShuffleboardTab tab = Shuffleboard.getTab(name);
    tab.addDouble("Drive Encoder Position Percent", module::getDriveEncoderPosition);
    tab.addDouble("Absolute Encoder Percent", module::getTurnEncoderValue);
    tab.addDouble("Drive Encoder Position Meters", module::getDriveEncoderPosition);
    tab.addDouble("Drive Encoder Velocity Meters", module::getDriveEncoderVelocity);
  }

  @Override
  public void periodic() {
      updateOdometry();
  }

  public void stopSystem() {
    m_frontLeft.stopSystem();
    m_frontRight.stopSystem();
    m_backLeft.stopSystem();
    m_backRight.stopSystem();
  }
}