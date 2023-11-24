// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.Util.AppliedEncoder;

public class SwerveModuleSystem extends SubsystemBase {
  private final double moduleMaxAngularVelocity = SwerveSystemConstants.maxAngularSpeed;
  private final double moduleMaxAngularAcceleration = SwerveSystemConstants.maxAngularAcceleration;
  private final int currentLimit = SwerveSystemConstants.swerveMotorCurrentLimit;

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final AppliedEncoder m_driveEncoder;
  private final AppliedEncoder m_turningEncoder;

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

  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(SwerveSystemConstants.turningFeedForward_S, SwerveSystemConstants.turningFeedForward_V);
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(SwerveSystemConstants.drivingFeedForward_S, SwerveSystemConstants.drivingFeedForward_V);

  public SwerveModuleSystem(
      int driveMotorID,
      int turningMotorID,
      int driveEncoderChannel,
      int turnEncoderChannel
    ) {
    m_driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

    m_driveMotor.setSmartCurrentLimit(currentLimit);
    m_turningMotor.setSmartCurrentLimit(currentLimit);

    m_driveEncoder = new AppliedEncoder(driveEncoderChannel);
    m_turningEncoder = new AppliedEncoder(turnEncoderChannel);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      m_driveEncoder.getRate(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getDistance(), new Rotation2d(m_turningEncoder.getDistance()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(m_turningEncoder.getDistance()));

    final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

    final double turnOutput =
        m_turningPIDController.calculate(m_turningEncoder.getDistance(), state.angle.getRadians());

    final double turnFeedforward =
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

    m_driveMotor.setVoltage(driveOutput + driveFeedforward);
    m_turningMotor.setVoltage(turnOutput + turnFeedforward);
  }

  public void stopSystem() {
    m_driveMotor.stopMotor();
    m_turningMotor.stopMotor();
  }
}
