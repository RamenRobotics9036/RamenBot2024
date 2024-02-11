// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.util.AppliedController;

/**
 * ArmSystem.
 */
public class ArmSystem extends SubsystemBase {

    private final CANSparkMax m_armMotorFollower = new CANSparkMax(ArmConstants.armMotorIDFollower,
            MotorType.kBrushless);
    private final CANSparkMax m_armMotorLeader = new CANSparkMax(ArmConstants.armMotorIDLeader,
            MotorType.kBrushless);
    private final DutyCycleEncoder m_ArmEncoder = new DutyCycleEncoder(
            ArmConstants.armEncoderChannel);
    private AppliedController m_controller;
    private RelativeEncoder m_relativeEncoder = m_armMotorLeader.getEncoder();

    private double maxOutputPercent = ArmConstants.maxOutputPercent;

    public ArmSystem(AppliedController controller) {
        m_armMotorLeader.restoreFactoryDefaults();
        m_armMotorFollower.restoreFactoryDefaults();
        m_armMotorLeader.setSmartCurrentLimit(ArmConstants.smartCurrentLimit);
        m_armMotorFollower.setSmartCurrentLimit(ArmConstants.smartCurrentLimit);

        m_armMotorLeader.setInverted(true);
        m_armMotorFollower.setInverted(true);

        m_armMotorLeader.setIdleMode(IdleMode.kBrake);
        m_armMotorFollower.setIdleMode(IdleMode.kBrake);
        m_armMotorFollower.follow(m_armMotorLeader);
        m_controller = controller;
        initShuffleBoard();
        setDefaultCommand(new ArmDefaultCommand(this, m_controller));

        m_relativeEncoder.setPositionConversionFactor((Math.PI * 2) / ArmConstants.gearRatio);
        m_relativeEncoder.setPosition(getArmAngleRadians());
        m_relativeEncoder
                .setVelocityConversionFactor(((Math.PI * 2) / ArmConstants.gearRatio) / 60);
    }

    public double getArmAngleRadians() {
        return (m_ArmEncoder.getAbsolutePosition() + ArmConstants.armAngleOffsetHorizontal) * 6;
    }

    public double getArmHeight() {
        return ArmConstants.pivotHeightOverGround +
                (ArmConstants.shootToPivotRadius * Math.sin(getArmAngleRadians()));
    }

    public double getShootingAngle(double distance) {
        return Math.atan(
                (ArmConstants.centerSpeakerHeight - ArmConstants.pivotHeightOverGround) / distance);

    }

    public void setArmSpeed(double speed) {
        speed = MathUtil.clamp(speed, -maxOutputPercent, maxOutputPercent);
        m_armMotorLeader.set(speed);
    }

    public double getRelativeEncoderRadians() {
        return Math.toRadians(m_relativeEncoder.getPosition());
    }

    @Override
    public void periodic() {
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Arm").addDouble("Arm Angle Absolute", () -> getArmAngleRadians());
        Shuffleboard.getTab("Arm").addDouble("Arm Height", () -> getArmHeight());
        Shuffleboard.getTab("Arm")
                .addDouble("Arm Angle Relative", () -> getRelativeEncoderRadians());
    }

    /**
     * Stop the arm system.
     */
    public void stopSystem() {
        m_armMotorLeader.stopMotor();
    }
}
