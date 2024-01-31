// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;

public class ShootingAngle extends CommandBase {
    /** Creates a new ShootingAngle. */

    private ArmSystem m_armSystem;
    private double m_angle;
    private double m_armLength;
    private double m_distance;

    public ShootingAngle(ArmSystem armSystem, double distance) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_armLength = 25.4;
        m_distance = distance;
        addRequirements(m_armSystem);

        // constructor

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_angle = Math.atan((80.25 - m_armLength) / m_distance);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
