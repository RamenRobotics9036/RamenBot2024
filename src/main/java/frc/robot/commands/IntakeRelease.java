package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;

public class IntakeRelease extends CommandBase {
    public IntakeSystem m_intake;
    public Timer m_timer = new Timer();

    public IntakeRelease(IntakeSystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= 5) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopSystem();
    }
}
