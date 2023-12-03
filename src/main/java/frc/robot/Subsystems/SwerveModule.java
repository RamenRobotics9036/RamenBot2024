// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.Util.AppliedEncoder;

public class SwerveModule {
  private static final double moduleMaxAngularVelocity = SwerveSystemConstants.maxAngularSpeed;
  private static final double moduleMaxAngularAcceleration = SwerveSystemConstants.maxAngularAcceleration;
  private static final int currentLimit = SwerveSystemConstants.swerveMotorCurrentLimit;
  private static final double maxOutput = MathUtil.clamp(SwerveSystemConstants.maxOutputPercentage, 0, 1);

  public static final double wheelRadius = SwerveSystemConstants.wheelRadiusMeters;
  public static final double driveGearRatio = SwerveSystemConstants.driveMotorGearBoxRatio;
  public static final double turnGearRatio = SwerveSystemConstants.turnMotorGearBoxRatio;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final AppliedEncoder m_turningAbsoluteEncoder;
  private final RelativeEncoder m_driveRelativeEncoder;

  private final double m_offSet;

  private final PIDController m_drivePIDController = new PIDController(
    SwerveSystemConstants.drivingPID_P,
    SwerveSystemConstants.drivingPID_I,
    SwerveSystemConstants.drivingPID_D
  );

  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          SwerveSystemConstants.turningPID_P,
          SwerveSystemConstants.turningPID_I,
          SwerveSystemConstants.turningPID_D,
          new TrapezoidProfile.Constraints(
              moduleMaxAngularVelocity,
              moduleMaxAngularAcceleration
          )
      );

  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveSystemConstants.drivingFeedForward_S, SwerveSystemConstants.drivingFeedForward_V);

  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int turnEncoderChannel,
      double offSet
    ) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    m_driveMotor.setSmartCurrentLimit(currentLimit);
    m_turningMotor.setSmartCurrentLimit(currentLimit);
    
    m_driveRelativeEncoder = m_driveMotor.getEncoder();
    m_turningAbsoluteEncoder = new AppliedEncoder(turnEncoderChannel);

    m_driveRelativeEncoder.setVelocityConversionFactor(wheelRadius * driveGearRatio * Math.PI * 2 / 60);
    m_driveRelativeEncoder.setPositionConversionFactor(wheelRadius * driveGearRatio * Math.PI * 2);

    m_turningAbsoluteEncoder.setDistancePerRotation(turnGearRatio * 2 * Math.PI);

    m_turningPIDController.enableContinuousInput(0, 2 * Math.PI);
    m_offSet = offSet;
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
    tab.addDouble("Desired Angle Setpoint", () -> m_turningPIDController.getSetpoint().position);
    tab.addDouble("Desired Velocty Setpoint", () -> m_drivePIDController.getSetpoint());
    tab.addDouble("Offset", () -> getOffset());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnEncoderValue()));

    final double driveOutput =
        m_drivePIDController.calculate(m_turningAbsoluteEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    final double turnOutput =
        m_turningPIDController.calculate(getTurnEncoderValue(), state.angle.getRadians());

    m_driveMotor.setVoltage((driveOutput + driveFeedforward) * maxOutput);
    m_turningMotor.setVoltage(turnOutput * maxOutput);
  }

  public double getOffset() {
    return m_offSet;
  }

  public double getTurnEncoderValue() {
    // TODO: use MathUtil unit conversion functions to make code more readable
    return (m_turningAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI - m_offSet) % (2 * Math.PI);
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
