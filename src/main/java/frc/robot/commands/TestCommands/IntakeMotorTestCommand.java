package frc.robot.commands.TestCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.IntakeSystem;

public class IntakeMotorTestCommand extends Command{
    public IntakeSystem m_intake;
    public Timer m_timer = new Timer();

    public double[] m_pos = new double[2];
    public boolean[] m_status = new boolean[2];

    public IntakeMotorTestCommand(IntakeSystem intake){
        m_intake = intake;
        addRequirements(m_intake);

        m_pos[0] = m_intake.getLeaderPosition();
        m_pos[1] = m_intake.getFollowerPosition();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_intake.setIntakeSpeed(TestConstants.testSpeed);

        if (MathUtil.applyDeadband(m_pos[0]-m_intake.getLeaderPosition(), TestConstants.errorMargin) != 0) {
            m_status[0]=true;
        }
        if (MathUtil.applyDeadband(m_pos[1]-m_intake.getFollowerPosition(), TestConstants.errorMargin) != 0) {
            m_status[1]=true;
        }
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= 0.5){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setStatus(m_status);
        m_intake.stopSystem();
    }
}