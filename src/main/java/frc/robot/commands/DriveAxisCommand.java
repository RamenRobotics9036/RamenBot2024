package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;

public class DriveAxisCommand extends CommandBase {
    protected SwerveDriveSystem m_swerveSystem;
    protected double m_xspeed;
    protected double m_yspeed;
    protected double m_rotspeed;
    protected double m_maxTime;
    protected Timer m_timer;

    public DriveAxisCommand(SwerveDriveSystem swerveSystem,
            double xspeed,
            double yspeed,
            double rotspeed,
            double maxTime) {
        m_swerveSystem = swerveSystem;
        m_xspeed = xspeed;
        m_yspeed = yspeed;
        m_rotspeed = rotspeed;
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
        m_swerveSystem.drive(m_xspeed, m_yspeed, m_rotspeed, true);
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
