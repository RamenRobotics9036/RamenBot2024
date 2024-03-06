package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.IntakeSystem;

public class IntakeMotorTestCommand extends Command {
    public IntakeSystem m_intake;
    public Timer m_timer = new Timer();

    public double[] m_oldPos = new double[2];
    public double[] m_currentPos = new double[2];
    public boolean[] m_status = new boolean[2];

    public String m_errorMessage;

    public IntakeMotorTestCommand(IntakeSystem intake) {
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_timer.start();
        m_oldPos[0] = m_intake.getLeaderPosition();
        m_oldPos[1] = m_intake.getFollowerPosition();
        m_currentPos[0] = m_intake.getLeaderPosition();
        m_currentPos[1] = m_intake.getFollowerPosition();
    }

    @Override
    public void execute() {
        m_intake.setIntakeSpeed(TestConstants.testSpeed);

        m_currentPos[0]=m_intake.getLeaderPosition() - m_oldPos[0];
        m_currentPos[1]=m_intake.getLeaderPosition() - m_oldPos[1];

        if (m_currentPos[0] >= TestConstants.errorMargin || m_currentPos[1] <= -TestConstants.errorMargin) {
            m_status[0] = true;
        }
        if (m_currentPos[1] >= TestConstants.errorMargin || m_currentPos[1] <= -TestConstants.errorMargin) {
            m_status[1] = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= 0.5) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!m_status[0]) {
            if (!m_status[1]) {
                m_errorMessage = "Both motors and/or encoders broken.";
            } else {
                m_errorMessage = "Leader motor and/or encoder is broken.";
            }
        } else {
            m_errorMessage = "Follower motor and/or encoder is broken.";
        }
        m_intake.setErrorMessage(m_errorMessage);
        m_intake.setStatus(m_status);
        m_intake.stopSystem();
    }
}