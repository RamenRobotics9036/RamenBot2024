package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;

public class PullIntakeCommand extends Command {
    private IntakeSystem m_intakeSystem;

    public PullIntakeCommand(IntakeSystem intakeSystem) {
        m_intakeSystem = intakeSystem;
        addRequirements(m_intakeSystem);
    }

    @Override
    public void initialize() {
        IntakeConstants.speed = IntakeConstants.intakeSpeed;
    }

    @Override
    public void execute() {
        m_intakeSystem.setOverride(true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSystem.stopSystem();
        IntakeConstants.speed = 0;
        m_intakeSystem.setOverride(false);
    }
}
