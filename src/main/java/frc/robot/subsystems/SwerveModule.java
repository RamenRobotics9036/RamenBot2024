// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.util.AppliedEncoder;

/**
 * SwerveModule.
 */
public class SwerveModule {
    private static final int currentLimit = SwerveSystemConstants.swerveMotorCurrentLimit;
    private static double maxOutput = MathUtil
            .clamp(SwerveSystemConstants.maxOutputPercentage, 0, 1);

    public static final double wheelRadius = SwerveSystemConstants.wheelRadiusMeters;
    public static final double driveGearRatio = SwerveSystemConstants.driveMotorGearBoxRatio;
    public static final double turnGearRatio = SwerveSystemConstants.turnMotorGearBoxRatio;

    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    private final AppliedEncoder m_turningAbsoluteEncoder;
    private final RelativeEncoder m_driveRelativeEncoder;
    private final RelativeEncoder m_turnRelativeEncoder;

    public static final double pidDriveP = SwerveSystemConstants.drivingPID_P;
    public static final double pidDriveI = SwerveSystemConstants.drivingPID_I;
    public static final double pidDriveD = SwerveSystemConstants.drivingPID_D;

    public static final double pidTurnP = SwerveSystemConstants.turningPID_P;
    public static final double pidTurnI = SwerveSystemConstants.turningPID_I;
    public static final double pidTurnD = SwerveSystemConstants.turningPID_D;

    private final double m_offSet;
    private double m_driveSetPoint = 0;
    private double m_turnSetPoint = 0;

    private final PIDController m_drivePidController = new PIDController(pidDriveP, pidDriveI,
            pidDriveD);

    private final SparkPIDController m_turnPidController;

    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(
            SwerveSystemConstants.drivingFeedForward_S, SwerveSystemConstants.drivingFeedForward_V);

    private SwerveModuleState m_swerveDesiredState;
    private SwerveModulePosition m_swerveDesiredPosition;

    /**
     * Constructor.
     */
    public SwerveModule(
            int driveMotorId,
            int turningMotorId,
            int turnEncoderChannel,
            double offSet) {
        m_offSet = offSet;
        m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_turningMotor.restoreFactoryDefaults();

        m_driveMotor.setIdleMode(IdleMode.kBrake);

        m_driveMotor.setSmartCurrentLimit(currentLimit);
        m_turningMotor.setSmartCurrentLimit(currentLimit);
        m_driveMotor.setInverted(false); // True for swerve A

        m_driveRelativeEncoder = m_driveMotor.getEncoder();
        m_turnRelativeEncoder = m_turningMotor.getEncoder();

        m_turningAbsoluteEncoder = new AppliedEncoder(turnEncoderChannel);

        m_driveRelativeEncoder
                .setVelocityConversionFactor(wheelRadius * driveGearRatio * Math.PI * 2 / 60);
        m_driveRelativeEncoder
                .setPositionConversionFactor(wheelRadius * driveGearRatio * Math.PI * 2);

        m_turnRelativeEncoder.setPositionConversionFactor((Math.PI * 2) / turnGearRatio);
        m_turnRelativeEncoder.setVelocityConversionFactor(((Math.PI * 2) / turnGearRatio) / 60);

        m_turnRelativeEncoder.setPosition(
                (m_turningAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI + m_offSet)
                        % (2 * Math.PI));
        m_turningAbsoluteEncoder.setDistancePerRotation(turnGearRatio * Math.PI * 2);

        m_turnPidController = m_turningMotor.getPIDController();
        m_turnPidController.setP(pidTurnP);
        m_turnPidController.setI(pidTurnI);
        m_turnPidController.setD(pidTurnD);

        m_turnPidController.setPositionPIDWrappingEnabled(true);
        m_turnPidController.setPositionPIDWrappingMinInput(0);
        m_turnPidController.setPositionPIDWrappingMaxInput(Math.PI * 2);

        m_swerveDesiredState = new SwerveModuleState(0,
                Rotation2d.fromRadians(getTurnEncoderValue()));
        m_swerveDesiredPosition = new SwerveModulePosition(0,
                Rotation2d.fromRadians(getTurnEncoderValue()));
    }

    public SwerveModuleState getStateActual() {
        return new SwerveModuleState(getDriveEncoderVelocity(),
                Rotation2d.fromRadians(getTurnEncoderValue()));
    }

    public SwerveModulePosition getPositionActual() {
        return new SwerveModulePosition(getDriveEncoderPosition(),
                Rotation2d.fromRadians(getTurnEncoderValue()));
    }

    public SwerveModuleState getState() {
        return getStateActual();
    }

    public SwerveModulePosition getPosition() {
        return getPositionActual();
    }

    public void toTeleop() {
        maxOutput = SwerveSystemConstants.maxOutputPercentage;
        updateDrivePid(11, 3);

    }

    public void toAuto() {
        maxOutput = SwerveSystemConstants.autoSpeed;
    }

    /**
     * Display the desired state of the swerve module to the dashboard.
     */
    public void displayDesiredStateToDashBoard(String tabName) {
        @SuppressWarnings("VariableDeclarationUsageDistance")
        ShuffleboardLayout drivingLayout = Shuffleboard.getTab(tabName).getLayout(
                "Driving",
                BuiltInLayouts.kList);

        @SuppressWarnings("VariableDeclarationUsageDistance")
        ShuffleboardLayout turningLayout = Shuffleboard.getTab(tabName).getLayout(
                "Turning",
                BuiltInLayouts.kList);

        ShuffleboardLayout pidTurningLayout = Shuffleboard.getTab(tabName)
                .getLayout("PID Tuning Turn", BuiltInLayouts.kList);
        ShuffleboardLayout pidDrivingLayout = Shuffleboard.getTab(tabName)
                .getLayout("PID Tuning Drive", BuiltInLayouts.kList);

        pidTurningLayout.addDouble("Desired Angle Setpoint Radians", () -> m_turnSetPoint);
        pidTurningLayout.addDouble("Absolute Encoder Radians", () -> getTurnEncoderRadians());

        pidDrivingLayout.addDouble("Desired Velocity Setpoint Rotations", () -> m_driveSetPoint);
        pidDrivingLayout.addDouble(
                "Drive Encoder Position Rotations",
                () -> getDriveEncoderPosition());

        drivingLayout.addDouble("Drive Encoder Velocity Meters", () -> getDriveEncoderVelocity());
        drivingLayout.addDouble(
                "Drive Encoder Position Rotations",
                () -> getDriveEncoderPosition());

        turningLayout.addDouble("Absolute Encoder Radians", () -> getTurnEncoderRadians());
        turningLayout.addDouble("Raw Absolute Encoder Radians", () -> getRawTurnEncoderRadians());
    }

    /**
     * Set the desired state of the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        m_swerveDesiredState = SwerveModuleState.optimize(
                desiredState,
                Rotation2d.fromRadians(getTurnEncoderValue()));

        m_swerveDesiredPosition.angle = Rotation2d.fromRadians(getTurnEncoderRadians());
        m_swerveDesiredPosition.distanceMeters += m_swerveDesiredState.speedMetersPerSecond * 0.02;

        final double driveOutput = m_drivePidController
                .calculate(
                        m_turningAbsoluteEncoder.getRate(),
                        m_swerveDesiredState.speedMetersPerSecond);

        final double driveFeedforward = m_driveFeedforward
                .calculate(m_swerveDesiredState.speedMetersPerSecond);
        m_turnPidController
                .setReference(m_swerveDesiredState.angle.getRadians(), ControlType.kPosition);

        double voltage = (driveOutput + driveFeedforward);
        voltage = MathUtil.clamp(voltage, -12 * maxOutput, 12 * maxOutput);
        m_driveMotor.setVoltage(voltage);

        m_driveSetPoint = m_swerveDesiredState.speedMetersPerSecond;
        m_turnSetPoint = m_swerveDesiredState.angle.getRadians();
    }

    public void updateDrivePid(double pidP, double pidD) {
        m_drivePidController.setP(pidP);
        m_drivePidController.setD(pidD);
    }

    public void updateTurnPid(double pidP, double pidD) {
        m_turnPidController.setP(pidP);
        m_turnPidController.setD(pidD);
    }

    public double getOffset() {
        return m_offSet;
    }

    public double getTurnEncoderValue() {
        return m_turnRelativeEncoder.getPosition();
    }

    public double getTurnEncoderRadians() {
        return m_turnRelativeEncoder.getPosition() % (Math.PI * 2);
    }

    public double getRawTurnEncoderRadians() {
        return (m_turningAbsoluteEncoder.getAbsolutePosition() * 2 * Math.PI + m_offSet)
                % (2 * Math.PI);
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
