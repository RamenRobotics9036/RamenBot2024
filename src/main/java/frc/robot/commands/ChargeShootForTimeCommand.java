package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSystem;

public class ChargeShootForTimeCommand extends Command {
    private ShooterSystem m_shooterSystem;
    private double m_time;
    private Timer m_timer;

    public ChargeShootForTimeCommand(ShooterSystem shooter, double time) {
        m_shooterSystem = shooter;
        m_time = time;
        addRequirements(m_shooterSystem);
    }

    @Override
    public void initialize() {
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_shooterSystem.setShootSpeed(ShooterConstants.shooterSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > m_time) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSystem.stopSystem();
    }
}
