// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * SwerveDriveSystem.
 */
public class ShooterSystem extends SubsystemBase {

    private final CANSparkMax m_shooterMotor = new CANSparkMax(ShooterConstants.shooterMotorID,
            MotorType.kBrushless);
    private double maxOutputPercent = ShooterConstants.maxOutputPercent;

    public ShooterSystem() {
        initShuffleBoard();
    }

    public void setShootSpeed(double speed) {
        speed = MathUtil.clamp(speed, -maxOutputPercent, maxOutputPercent);
        m_shooterMotor.set(speed);
    }

    public double getShootSpeed() {
        return m_shooterMotor.get();
    }

    public void initShuffleBoard() {
        Shuffleboard.getTab("Shooter").addDouble("Shooter Speed: ", () -> getShootSpeed());
    }

    @Override
    public void periodic() {
    }

    /**
     * Stop the swerve drive system.
     */
    public void stopSystem() {

        m_shooterMotor.stopMotor();
    }
}
