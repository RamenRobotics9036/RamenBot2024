package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;

public class PullBackShooterCommand extends Command {
    private ShooterSystem m_shooterSystem;
    private Timer m_timer;

    public PullBackShooterCommand(ShooterSystem intakeSystem) {
        m_shooterSystem = intakeSystem;
        addRequirements(m_shooterSystem);
    }

    @Override
    public void initialize() {
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        // In case the motor is stil spinning in wrong direction on startup
        m_shooterSystem.setShootSpeed(-0.1);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= 0.5) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSystem.stopSystem();
    }
}
