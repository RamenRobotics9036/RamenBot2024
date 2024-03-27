package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class ShootCommand extends Command {
    private IntakeSystem m_intakeSystem;
    private ShooterSystem m_shooterSystem;

    private Timer m_timer = new Timer();

    public ShootCommand(IntakeSystem intake, ShooterSystem shooter) {
        m_intakeSystem = intake;
        m_shooterSystem = shooter;
        addRequirements(m_intakeSystem, m_shooterSystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_shooterSystem.setShootSpeed(ShooterConstants.shooterSpeed);
        m_intakeSystem.setIntakeSpeed(IntakeConstants.intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        // 2 is a dummy value. Please change it or divide speed by something later.
        if (m_timer.get() >= 2) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSystem.stopSystem();
        m_intakeSystem.stopSystem();
    }
}
