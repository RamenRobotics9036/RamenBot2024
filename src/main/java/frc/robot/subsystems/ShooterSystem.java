// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * SwerveDriveSystem.
 */
public class ShooterSystem extends SubsystemBase {
    private CANSparkMax m_shooterMotorFollower = new CANSparkMax(
            ShooterConstants.shooterRightMotorID,
            MotorType.kBrushless);
    private CANSparkMax m_shooterMotorLeader = new CANSparkMax(ShooterConstants.shooterLeftMotorID,
            MotorType.kBrushless);
    private RelativeEncoder m_encoder = m_shooterMotorLeader.getEncoder();

    private double maxOutputPercent = ShooterConstants.maxOutputPercent;

    public ShooterSystem() {
        m_shooterMotorLeader.setInverted(false);
        m_shooterMotorFollower.setInverted(true);
        m_shooterMotorFollower.follow(m_shooterMotorLeader);

        m_shooterMotorLeader.setIdleMode(IdleMode.kBrake);
        m_shooterMotorFollower.setIdleMode(IdleMode.kBrake);
        // initShuffleBoard();
    }

    public void setShootSpeed(double speed) {
        speed = MathUtil.clamp(speed, -maxOutputPercent, maxOutputPercent);
        m_shooterMotorLeader.set(speed);
    }

    public double getShootSpeed() {
        return m_shooterMotorLeader.get();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Shooter").addDouble("Shooter Speed: ", () -> getShootSpeed());
    }

    public double getAngle() {
        return m_encoder.getPosition();
    }

    @Override
    public void periodic() {
    }

    /**
     * Stop the swerve drive system.
     */
    public void stopSystem() {
        m_shooterMotorLeader.stopMotor();
    }
}
