// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveSystemConstants;
import frc.robot.Constants.SwerveSystemConstants.SwerveSystemDeviceConstants;
import frc.robot.commands.DriveSwerveCommand;
import frc.robot.util.AppliedController;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * SwerveDriveSystem.
 */
public class ShooterSystem extends SubsystemBase {

    public final CANSparkMax m_intakeMotor = new CANSparkMax(ShooterConstants.intakeMotorID,
            MotorType.kBrushless);
    public final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.shooterMotorID,
            MotorType.kBrushless);
    public double speed;

    public ShooterSystem() {

    }

    public void setShootSpeed(double SPEED) {
        speed = SPEED;
        m_shooterMotor.set(speed);

    }

    public double getShootSpeed() {
        return m_shooterMotor.get();
    }

    public void setIntakeSpeed(double SPEED) {
        speed = SPEED;
        m_intakeMotor.set(speed);
    }

    public double getIntakeSpeed() {
        return m_intakeMotor.get();

    }

    @Override
    public void periodic() {
        // Shuffleboard.getTab("Sensor").addBoolean(getName(), null);
        // Shuffleboard.getTab("Swerve").add("X Pose Meters", m_odometry.getPoseMeters().getX())

    }

    /**
     * Stop the swerve drive system.
     */
    public void stopSystem() {
        m_intakeMotor.stopMotor();
        m_shooterMotor.stopMotor();
    }
}
