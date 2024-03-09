package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class SetShooterSpeedCommand extends Command {
    private ShooterSystem m_shooterSystem;
    private double m_maxTime;
    private double m_speed;
    private Timer m_timer;

    public SetShooterSpeedCommand(ShooterSystem shooterSystem, double maxTime, double speed) {
        m_shooterSystem = shooterSystem;
        m_maxTime = maxTime;
        m_speed = speed;
        addRequirements(m_shooterSystem);
    }

    @Override
    public void initialize() {
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_shooterSystem.setShootSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        // $TODO - Remove this
        if (RobotBase.isSimulation()) {
            return true;
        }

        if (m_timer.get() >= m_maxTime) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSystem.stopSystem();
    }
}
