package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class PushBackCommand extends Command {
    private IntakeSystem m_IntakeSystem;
    private ShooterSystem m_shooterSystem;
    private Timer m_timer = new Timer();

    private PullIntakeCommand m_cmd = new PullIntakeCommand(m_IntakeSystem);

    public PushBackCommand(ShooterSystem shooter, IntakeSystem intake) {
        m_shooterSystem = shooter;
        m_IntakeSystem = intake;
        addRequirements(m_shooterSystem, m_IntakeSystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_shooterSystem.setShootSpeed(-ShooterConstants.shooterSpeed);
    }

    @Override
    public boolean isFinished() {
        // 0.5 is a dummy value. Please change it or divide speed by something later.
        if (m_timer.get() >= 0.5) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_cmd.schedule();
    }
}
