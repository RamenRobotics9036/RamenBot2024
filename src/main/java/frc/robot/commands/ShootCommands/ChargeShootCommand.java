package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class ChargeShootCommand extends Command {
    private IntakeSystem m_intakeSystem;
    private ShooterSystem m_shooterSystem;
    private PullBackCommand m_pullCmd = new PullBackCommand(m_intakeSystem);
    private ShootCommand m_shootCmd = new ShootCommand(m_intakeSystem, m_shooterSystem);

    public ChargeShootCommand(IntakeSystem intake, ShooterSystem shooter) {
        m_intakeSystem = intake;
        m_shooterSystem = shooter;
        addRequirements(m_intakeSystem, m_shooterSystem);
    }

    @Override
    public void initialize() {
        m_pullCmd.schedule();
    }

    @Override
    public void execute() {
        m_shooterSystem.setShootSpeed(ShooterConstants.shooterSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shootCmd.schedule();
    }
}
