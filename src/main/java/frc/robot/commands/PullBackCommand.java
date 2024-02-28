package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;

public class PullBackCommand extends Command {
    private IntakeSystem m_intakeSystem;
    private double m_angle;

    public PullBackCommand(IntakeSystem intakeSystem) {
        m_intakeSystem = intakeSystem;
        addRequirements(m_intakeSystem);
    }

    @Override
    public void initialize() {
        m_angle = m_intakeSystem.getIntakeAngle();
        SmartDashboard.putNumber("Start Pos Intake", m_angle);
        IntakeConstants.speed = IntakeConstants.pullBackSpeed;
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (m_intakeSystem.getIntakeAngle() > m_angle + IntakeConstants.pullBackAmount) {
            SmartDashboard.putNumber("End Pos Intake", m_intakeSystem.getIntakeAngle());
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSystem.stopSystem();
        IntakeConstants.speed = 0;
    }
}
