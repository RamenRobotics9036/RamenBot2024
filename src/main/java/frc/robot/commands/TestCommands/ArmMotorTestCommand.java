package frc.robot.commands.TestCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.ArmSystem;

public class ArmMotorTestCommand extends Command {
    public ArmSystem m_arm;

    public Timer m_timer = new Timer();

    public double[] m_pos = new double[2];
    public boolean[] m_status = new boolean[2];

    public String m_errorMessage;

    public ArmMotorTestCommand(ArmSystem arm) {
        m_arm = arm;
        addRequirements(m_arm);

        m_pos[0] = m_arm.getLeaderEncoderRadians();
        m_pos[1] = m_arm.getFollowerEncoderRadians();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_arm.setArmSpeed(TestConstants.testSpeed);

        if (m_pos[0] - m_arm.getLeaderEncoderRadians() <= -TestConstants.errorMargin
                || m_pos[0] - m_arm.getLeaderEncoderRadians() >= TestConstants.errorMargin) {
            m_status[0] = true;
        }

        if (m_pos[1] - m_arm.getFollowerEncoderRadians() <= -TestConstants.errorMargin
                || m_pos[1] - m_arm.getFollowerEncoderRadians() >= TestConstants.errorMargin) {
            m_status[2] = true;
        }
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= 0.5 || (m_status[0] == true && m_status[1] == true)) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (!m_status[0]) {
            if (!m_status[1]) {
                m_errorMessage = "Unfortunately, both motors and/or encoders are broken." + "\n";
            } else {
                m_errorMessage = "Leader motor and/or encoder is broken." + "\n";
            }
        } else {
            if (!m_status[1]) {
                m_errorMessage = "Follower motor and/or encoder is broken." + "\n";
            } else {
                m_errorMessage = "The arm is working! Good job!." + "\n";
            }
        }
        m_errorMessage += "Leader motor moved " + String.valueOf(m_pos[0]) + "." + "\n";
        m_errorMessage += "Follower motor moved " + String.valueOf(m_pos[1]) + "." + "\n";
        m_errorMessage += "Predicted was " + String.valueOf(TestConstants.armMotorPredicted) + ".";
        m_arm.setStatus(m_status);
        m_arm.setErrorMessage(m_errorMessage);
        m_arm.stopSystem();
    }
}