package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;

public class DriveRotCommand extends CommandBase {
    protected SwerveDriveSystem m_swerveSystem;
    protected double m_speed;
    protected double m_maxTime;
    protected Timer m_timer;

    public DriveRotCommand(SwerveDriveSystem swerveSystem, double speed, double maxTime) {
        m_swerveSystem = swerveSystem;
        m_speed = speed;
        m_timer = new Timer();
        m_maxTime = maxTime;
        addRequirements(m_swerveSystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_swerveSystem.drive(0, 0, m_speed, true);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= m_maxTime) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSystem.stopSystem();
    }
}
