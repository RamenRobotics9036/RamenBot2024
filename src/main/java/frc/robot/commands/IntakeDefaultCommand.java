package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;

public class IntakeDefaultCommand extends Command {
    private IntakeSystem m_intakeSystem;

    public IntakeDefaultCommand(IntakeSystem intakeSystem) {
        m_intakeSystem = intakeSystem;
        addRequirements(m_intakeSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intakeSystem.setIntakeSpeed(-IntakeConstants.intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSystem.stopSystem();
    }
}
