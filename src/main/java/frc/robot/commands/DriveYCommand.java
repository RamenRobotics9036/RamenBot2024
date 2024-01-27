package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;

public class DriveYCommand extends CommandBase {
    private SwerveDriveSystem m_swerveSystem;
    private double m_speed;
    private double m_maxTime;
    private Timer m_timer;

    public DriveYCommand(SwerveDriveSystem swerveSystem, double speed, double maxSpeed) {
        m_swerveSystem = swerveSystem;
        m_speed = speed;
        m_timer = new Timer();
        m_maxTime = maxSpeed;
        addRequirements(m_swerveSystem);
    }


    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_swerveSystem.drive(0, m_speed, 0, true);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= m_maxTime){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSystem.stopSystem();
    }
}
