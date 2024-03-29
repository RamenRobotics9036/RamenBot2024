package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RevConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.util.AppliedController;

public class IntakeRevCommand extends Command {
    private ShooterSystem m_shooterSystem;
    private IntakeSystem m_intakeSystem;
    private Timer m_timer;
    private AppliedController m_controller;

    public IntakeRevCommand(
            IntakeSystem intakeSystem,
            ShooterSystem shooterSystem,
            AppliedController controller) {
        m_intakeSystem = intakeSystem;
        m_shooterSystem = shooterSystem;
        m_controller = controller;

        addRequirements(m_intakeSystem, m_shooterSystem);
    }

    @Override
    public void initialize() {
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_timer.get() >= RevConstants.revTime) {
            m_intakeSystem.setIntakeSpeed(-IntakeConstants.intakeSpeed);
        }
        m_shooterSystem.setShootSpeed(ShooterConstants.shooterSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= RevConstants.maxTime) {
            return true;
        }
        if (m_controller.commandCancel()) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSystem.stopSystem();
        m_shooterSystem.stopSystem();
    }
}
