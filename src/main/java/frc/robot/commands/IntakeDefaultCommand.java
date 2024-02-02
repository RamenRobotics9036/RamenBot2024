package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;

public class IntakeDefaultCommand extends CommandBase {
    public IntakeSystem m_intake;

    public IntakeDefaultCommand(IntakeSystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_intake.getReflectometer()) {
            m_intake.setIntakeSpeed(0);
        }
        else {
            m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopSystem();
    }
}
