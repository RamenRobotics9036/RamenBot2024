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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
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
public class ArmAndIntakeSystem extends SubsystemBase {

    public final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID,
            MotorType.kBrushless);
    public final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.armMotorID,
            MotorType.kBrushless);
    public final DutyCycleEncoder m_ArmEncoder = new DutyCycleEncoder(
            ArmConstants.armEncoderChannel);
    // public DigitalInput refelectometer = new DigitalInput(ShooterConstants.reflectChannel);
    public double Speed;

    public ArmAndIntakeSystem() {
        initShuffleBoard();

    }

    public void setIntakeSpeed(double SPEED) {
        Speed = SPEED;
        m_intakeMotor.set(Speed);
    }

    public double getIntakeSpeed() {
        return m_intakeMotor.get();

    }

    public double getArmangle() {
        return m_ArmEncoder.getAbsolutePosition();

    }

    @Override
    public void periodic() {
        // Shuffleboard.getTab("Sensor").addBoolean(getName(), null);
        // Shuffleboard.getTab("Swerve").add("X Pose Meters", m_odometry.getPoseMeters().getX())

    }

    public void initShuffleBoard() {

        Shuffleboard.getTab("Arm and Intake").add("Arm Angle: ",
                m_ArmEncoder.getAbsolutePosition());
        Shuffleboard.getTab("Arm and Intake").add("Intake Speed: ", getIntakeSpeed());

    }

    /**
     * Stop the swerve drive system.
     */
    public void stopSystem() {
        m_intakeMotor.stopMotor();
        m_armMotor.stopMotor();
    }
}
