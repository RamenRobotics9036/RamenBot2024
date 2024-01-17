// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

/**
 * SwerveDriveSystem.
 */
public class ArmAndIntakeSystem extends SubsystemBase {

    public final CANSparkMax m_intakeMotor = new CANSparkMax(IntakeConstants.intakeMotorID,
            MotorType.kBrushless);
    public final CANSparkMax m_armMotor = new CANSparkMax(ArmConstants.armMotorID,
            MotorType.kBrushless);
    public final DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(
            ArmConstants.armEncoderChannel);
    // public DigitalInput refelectometer = new DigitalInput(ShooterConstants.reflectChannel);
    public double m_speed;

    public ArmAndIntakeSystem() {
        initShuffleBoard();

    }

    public void setIntakeSpeed(double speed) {
        m_speed = speed;
        m_intakeMotor.set(m_speed);
    }

    public double getIntakeSpeed() {
        return m_intakeMotor.get();

    }

    public double getArmangle() {
        return m_armEncoder.getAbsolutePosition();

    }

    @Override
    public void periodic() {
        // Shuffleboard.getTab("Sensor").addBoolean(getName(), null);
        // Shuffleboard.getTab("Swerve").add("X Pose Meters", m_odometry.getPoseMeters().getX())

    }

    /**
     * Initialize the Shuffleboard.
     */
    public void initShuffleBoard() {

        Shuffleboard.getTab("Arm and Intake").add("Arm Angle: ",
                m_armEncoder.getAbsolutePosition());
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
