// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CommandsConstants.SetArmConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.util.AppliedController;
import frc.robot.util.TargetAngle;

/**
 * ArmSystem.
 */
public class ArmSystem extends SubsystemBase {
    private final CANSparkMax m_armMotorFollower = new CANSparkMax(ArmConstants.armMotorIDFollower,
            MotorType.kBrushless);
    private final CANSparkMax m_armMotorLeader = new CANSparkMax(ArmConstants.armMotorIDLeader,
            MotorType.kBrushless);
    private final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(
            ArmConstants.armEncoderChannel);
    private AppliedController m_controller;
    private RelativeEncoder m_relativeEncoder = m_armMotorLeader.getEncoder();

    private final TargetAngle m_targetAngle = new TargetAngle(VisionConstants.angleLookUpTable);

    private double m_maxOutputPercent = ArmConstants.maxOutputPercent;

    public ArmSystem(AppliedController controller) {
        m_armMotorLeader.restoreFactoryDefaults();
        m_armMotorFollower.restoreFactoryDefaults();
        m_armMotorLeader.setSmartCurrentLimit(ArmConstants.smartCurrentLimit);
        m_armMotorFollower.setSmartCurrentLimit(ArmConstants.smartCurrentLimit);

        m_ArmEncoder.setPositionOffset(0.14);

        m_armMotorLeader.setInverted(true);
        m_armMotorFollower.setInverted(true);

        m_armMotorLeader.setIdleMode(IdleMode.kBrake);
        m_armMotorFollower.setIdleMode(IdleMode.kBrake);
        m_armMotorFollower.follow(m_armMotorLeader);
        m_controller = controller;
        // initShuffleBoard();
        Shuffleboard.getTab("Arm").addDouble("Arm Angle Absolute", () -> getArmAngleRadians());
        Shuffleboard.getTab("Arm").addDouble("Arm Angle Zero To One", () -> getArmAngleZeroToOne());
        setDefaultCommand(new ArmDefaultCommand(this, m_controller));
        m_ArmEncoder.setPositionOffset(0.14);
        m_relativeEncoder.setPositionConversionFactor((Math.PI * 2) / ArmConstants.gearRatio);
        m_relativeEncoder.setPosition(getArmAngleRadians());
    }

    public void toTeleop() {
        m_maxOutputPercent = ArmConstants.maxOutputPercentTeleop;
    }

    public void toAuto() {
        m_maxOutputPercent = ArmConstants.maxOutputPercent;
    }

    public double getArmAngleRadians() {
        return 2 * Math.PI
                - (m_armEncoder.getAbsolutePosition() + ArmConstants.armAngleOffsetHorizontal) * 6;
    }

    public double getArmAngleZeroToOne() {
        return (Constants.CommandsConstants.SetArmConstants.armMin - 6.28)
                / (m_ArmEncoder.getAbsolutePosition() * 6);

    }

    public double getArmHeight() {
        return ArmConstants.pivotHeightOverGround
                + (ArmConstants.shootToPivotRadius * Math.sin(getArmAngleRadians()));
    }

    public double getShootingAngle(double distance) {
        return m_targetAngle.getShootingAngle(distance);
    }

    public void setArmSpeed(double speed) {
        speed = MathUtil.clamp(speed, -m_maxOutputPercent, m_maxOutputPercent);
        if ((speed < 0 && getArmAngleRadians() < SetArmConstants.armMin)
                || (speed > 0 && getArmAngleRadians() > SetArmConstants.armMax)) {
            m_armMotorLeader.set(speed);
        }
        else {
            m_armMotorLeader.set(0);
        }
    }

    public void setArmSpeedAdmin(double speed) {
        speed = MathUtil.clamp(speed, -m_maxOutputPercent, m_maxOutputPercent);
        m_armMotorLeader.set(speed);
    }

    private double getRelativeEncoderRadians() {
        return m_relativeEncoder.getPosition();
    }

    public double getArmSpeed() {
        return m_armMotorLeader.get();
    }

    @Override
    public void periodic() {
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Arm").addDouble("Arm Angle Absolute", () -> getArmAngleRadians());
        Shuffleboard.getTab("Arm").addDouble("Arm Height", () -> getArmHeight());
        Shuffleboard.getTab("Arm").addDouble("Arm Speed", () -> getArmSpeed());
        Shuffleboard.getTab("Arm")
                .addDouble("Arm Angle Relative", () -> getRelativeEncoderRadians());
        // Shuffleboard.getTab("Arm").addDouble("Arm Angle Zero To One", () ->
        // getArmAngleZeroToOne());
    }

    /**
     * Stop the arm system.
     */
    public void stopSystem() {
        m_armMotorLeader.stopMotor();
    }
}
