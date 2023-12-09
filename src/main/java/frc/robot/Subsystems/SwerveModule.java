// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.Util.AppliedEncoder;

public class SwerveModule {
  private static final int currentLimit = SwerveSystemConstants.swerveMotorCurrentLimit;
  private static final double maxOutput = MathUtil.clamp(SwerveSystemConstants.maxOutputPercentage, 0, 1);

  public static final double wheelRadius = SwerveSystemConstants.wheelRadiusMeters;
  public static final double driveGearRatio = SwerveSystemConstants.driveMotorGearBoxRatio;
  public static final double turnGearRatio = SwerveSystemConstants.turnMotorGearBoxRatio;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final AppliedEncoder m_turningAbsoluteEncoder;
  private final RelativeEncoder m_driveRelativeEncoder;
  private final RelativeEncoder m_turnRelativeEncoder;

  private final double m_offSet;
  private double m_driveSetPoint = 0;
  private double m_turnSetPoint = 0;

  private final PIDController m_drivePIDController = new PIDController(
    SwerveSystemConstants.drivingPID_P,
    SwerveSystemConstants.drivingPID_I,
    SwerveSystemConstants.drivingPID_D);
    
  private final SparkMaxPIDController m_turnPIDController;

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveSystemConstants.drivingFeedForward_S, SwerveSystemConstants.drivingFeedForward_V);

  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turnEncoderChannel,
      double offSet
    ) {
    m_offSet = offSet;
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
    
    m_driveMotor.restoreFactoryDefaults();
    m_turningMotor.restoreFactoryDefaults();
    m_turningMotor.setInverted(true);

    m_driveMotor.setSmartCurrentLimit(currentLimit);
    m_turningMotor.setSmartCurrentLimit(currentLimit);
    
    m_driveRelativeEncoder = m_driveMotor.getEncoder();
    m_turnRelativeEncoder = m_turningMotor.getEncoder();
    m_turningAbsoluteEncoder = new AppliedEncoder(turnEncoderChannel);

    m_driveRelativeEncoder.setVelocityConversionFactor(wheelRadius * driveGearRatio * Math.PI * 2 / 60);
    m_driveRelativeEncoder.setPositionConversionFactor(wheelRadius * driveGearRatio * Math.PI * 2);

    m_turnRelativeEncoder.setPositionConversionFactor((Math.PI * 2) / turnGearRatio);
    m_turnRelativeEncoder.setVelocityConversionFactor(((Math.PI * 2) / turnGearRatio) / 60);

    m_turnRelativeEncoder.setPosition((m_turningAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI - m_offSet) % (2 * Math.PI));
    m_turningAbsoluteEncoder.setDistancePerRotation(turnGearRatio * Math.PI * 2);

    m_turnPIDController = m_turningMotor.getPIDController();
    m_turnPIDController.setP(SwerveSystemConstants.turningPID_P);
    m_turnPIDController.setI(SwerveSystemConstants.turningPID_I);
    m_turnPIDController.setD(SwerveSystemConstants.turningPID_D);

    m_turnPIDController.setPositionPIDWrappingEnabled(true);
    m_turnPIDController.setPositionPIDWrappingMinInput(0);
    m_turnPIDController.setPositionPIDWrappingMaxInput(Math.PI * 2);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveEncoderVelocity(), new Rotation2d(getTurnEncoderValue()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      getDriveEncoderPosition(), new Rotation2d(getTurnEncoderValue()));
  }

  public void displayDesiredStateToDashBoard(String tabName) {
    ShuffleboardTab tab = Shuffleboard.getTab(tabName);
    tab.addDouble("Desired Angle Setpoint", () -> m_turnSetPoint);
    tab.addDouble("Desired Velocty Setpoint", () -> m_driveSetPoint);
    tab.addDouble("Offset", () -> getOffset());

    tab.addDouble("Drive Encoder Position Percent", () -> getDriveEncoderPosition());
    tab.addDouble("Absolute Encoder Percent", () -> getTurnEncoderValue());
    tab.addDouble("Drive Encoder Position Meters", () -> getDriveEncoderPosition());
    tab.addDouble("Drive Encoder Velocity Meters", () -> getDriveEncoderVelocity());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurnEncoderValue()));

    final double driveOutput =
        m_drivePIDController.calculate(m_turningAbsoluteEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    m_turnPIDController.setReference(state.angle.getRadians(), ControlType.kPosition);

    m_driveMotor.setVoltage((driveOutput + driveFeedforward) * maxOutput);  
    
    m_driveSetPoint = state.speedMetersPerSecond;
    m_turnSetPoint = state.angle.getRadians();
  }

  public double getOffset() {
    return m_offSet;
  }

  public double getTurnEncoderValue() {
    return m_turnRelativeEncoder.getPosition();
  }

  public double getDriveEncoderPosition() {
    return m_driveRelativeEncoder.getPosition();
  }

  public double getDriveEncoderVelocity() {
    return m_driveRelativeEncoder.getVelocity();
  }

  public void stopSystem() {
    m_driveMotor.stopMotor();
    m_turningMotor.stopMotor();
  }
}
