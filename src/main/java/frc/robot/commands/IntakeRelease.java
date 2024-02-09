package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.CommandsConstants.IntakeReleaseConstants;
import frc.robot.subsystems.IntakeSystem;

public class IntakeRelease extends Command {
    private IntakeSystem m_intakeSystem;
    private Timer m_timer;

    public IntakeRelease(IntakeSystem intakeSystem) {
        m_intakeSystem = intakeSystem;
        m_timer = new Timer();
        addRequirements(m_intakeSystem);
    }

    @Override
    public void initialize() {
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_intakeSystem.setIntakeSpeed(IntakeConstants.intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= IntakeReleaseConstants.maxTime) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSystem.stopSystem();
    }
}
