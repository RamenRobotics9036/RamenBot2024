package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RevConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.util.AppliedController;

public class ShootCommandTele extends Command {
    private IntakeSystem m_intakeSystem;
    private Timer m_timer;
    private AppliedController m_controller;

    public ShootCommandTele(
            IntakeSystem intakeSystem,
            AppliedController controller) {
        m_intakeSystem = intakeSystem;
        m_controller = controller;

        addRequirements(m_intakeSystem);
    }

    @Override
    public void initialize() {
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_intakeSystem.setIntakeSpeed(IntakeConstants.maxOutputPercent);
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
        IntakeConstants.speed = -IntakeConstants.intakeSpeed;
        ShooterConstants.shouldCharge = false;
    }
}
