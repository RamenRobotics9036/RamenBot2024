// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    private final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.armMotorID,
            MotorType.kBrushless);
    private final DutyCycleEncoder m_ArmEncoder = new DutyCycleEncoder(
            ArmConstants.armEncoderChannel);
    private AppliedController m_controller;

    private double maxOutputPercent = ArmConstants.maxOutputPercent;

    public ArmSystem(AppliedController controller) {
        m_controller = controller;
        initShuffleBoard();
        setDefaultCommand(new ArmDefaultCommand(this, m_controller));
    }

    public double getArmAngleRadians() {
        return m_ArmEncoder.getAbsolutePosition();
    }

    public double getArmHeight() {
        return ArmConstants.pivotHeightOverGround + ArmConstants.shootToPivotRadius
                * Math.sin(
                        Math.toRadians(
                                getArmAngleRadians() + ArmConstants.armAngleOffsetHorizontal));
    }

    public double getShootingAngle(
            double m_distance,
            double m_armLength,
            double m_centerSpeakerHeight) {
        return Math.atan((m_centerSpeakerHeight - m_armLength) / m_distance);

    }

    public void setArmSpeed(double speed) {
        speed = MathUtil.clamp(speed, -maxOutputPercent, maxOutputPercent);
        m_armMotor.set(speed);
    }

    @Override
    public void periodic() {
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Arm").addDouble("Arm Angle", () -> getArmAngleRadians());
        Shuffleboard.getTab("Arm").addDouble("Arm Height", () -> getArmHeight());
    }

    /**
     * Stop the arm system.
     */
    public void stopSystem() {
        m_armMotor.stopMotor();
    }
}
