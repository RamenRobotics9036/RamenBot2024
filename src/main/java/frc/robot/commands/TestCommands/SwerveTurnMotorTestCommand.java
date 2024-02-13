package frc.robot.commands.TestCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.SwerveDriveSystem;

public class SwerveTurnMotorTestCommand extends Command{
    public SwerveDriveSystem m_swerve;
    public Timer m_timer = new Timer();

    public double m_ySpeed = 0.25;
    public double m_xSpeed = 0.25;
    public double[] m_pos = new double[4];
    public boolean[] m_status = new boolean[4];

    public SwerveTurnMotorTestCommand(SwerveDriveSystem swerve){
        m_swerve = swerve;
        addRequirements(swerve);

        m_pos[0] = m_swerve.getFrontLeftTurnEncoder();
        m_pos[1] = m_swerve.getBackLeftTurnEncoder();
        m_pos[2] = m_swerve.getFrontRightTurnEncoder();
        m_pos[3] = m_swerve.getBackRightTurnEncoder();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_swerve.drive(m_xSpeed, m_ySpeed, 0);
        m_xSpeed+=0.01;
        m_ySpeed-=0.01;

        if (MathUtil.applyDeadband(m_pos[0]-m_swerve.getFrontLeftTurnEncoder(), TestConstants.errorMargin) != 0) {
            m_status[0]=true;
        }
        if (MathUtil.applyDeadband(m_pos[1]-m_swerve.getBackLeftTurnEncoder(), TestConstants.errorMargin) != 0) {
            m_status[1]=true;
        }
        if (MathUtil.applyDeadband(m_pos[2]-m_swerve.getFrontRightTurnEncoder(), TestConstants.errorMargin) != 0) {
            m_status[2]=true;
        }
        if (MathUtil.applyDeadband(m_pos[3]-m_swerve.getBackRightTurnEncoder(), TestConstants.errorMargin) != 0) {
            m_status[3]=true;
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
        m_swerve.setStatus(m_status);
        m_swerve.stopSystem();
    }
}