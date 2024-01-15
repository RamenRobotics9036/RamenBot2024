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
 // $TODO - Copy this to a new file called ShooterSystem.java.  And in that file, rename the class
public class ShooterSystem extends SubsystemBase {


    public final CANSparkMax m_max = new CANSparkMax(ShooterConstants.shooterMotorID, MotorType.kBrushless);
    public double speed;
    public ShooterSystem() {
      
    }
    public void setShootSpeed(double SPEED) {
        speed = SPEED;
        m_max.set(speed);
      
    }
    public double getShootSpeed() {
        return speed;
    }

  


    @Override
    public void periodic() {
    
        // Shuffleboard.getTab("Swerve").add("X Pose Meters", m_odometry.getPoseMeters().getX())

    }

    /**
     * Stop the swerve drive system.
     */
    public void stopSystem() {
    m_max.stopMotor();
    }
}
