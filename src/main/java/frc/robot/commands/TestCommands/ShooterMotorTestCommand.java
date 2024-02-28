package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.ShooterSystem;

public class ShooterMotorTestCommand extends Command {
    public ShooterSystem m_shooter;
    public Timer m_timer = new Timer();

    public double[] m_pos = new double[2];
    public boolean[] m_status = new boolean[2];

    public ShooterMotorTestCommand(ShooterSystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);

        m_pos[0] = m_shooter.getLeaderPosition();
        m_pos[1] = m_shooter.getFollowerPosition();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_shooter.setShootSpeed(TestConstants.testSpeed);

        if (MathUtil.applyDeadband(m_pos[0] - m_shooter.getLeaderPosition(), TestConstants.errorMargin) != 0) {
            m_status[0] = true;
        }
        if (MathUtil.applyDeadband(m_pos[1] - m_shooter.getFollowerPosition(), TestConstants.errorMargin) != 0) {
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
        m_shooter.setStatus(m_status);
        m_shooter.stopSystem();
    }
}