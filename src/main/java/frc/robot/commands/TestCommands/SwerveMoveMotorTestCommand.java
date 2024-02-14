package frc.robot.commands.TestCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.SwerveDriveSystem;

public class SwerveMoveMotorTestCommand extends Command{
    public SwerveDriveSystem m_swerve;
    public Timer m_timer = new Timer();

    public double m_ySpeed = 0.25;
    public double m_xSpeed = 0.25;
    public double[] m_pos = new double[4];
    public boolean[] m_status = new boolean[4];

    public SwerveMoveMotorTestCommand(SwerveDriveSystem swerve){
        m_swerve = swerve;
        addRequirements(swerve);

        m_pos[0] = m_swerve.getFrontLeftDriveEncoder();
        m_pos[1] = m_swerve.getBackLeftDriveEncoder();
        m_pos[2] = m_swerve.getFrontRightDriveEncoder();
        m_pos[3] = m_swerve.getBackRightDriveEncoder();
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

        if (MathUtil.applyDeadband(m_pos[0]-m_swerve.getFrontLeftDriveEncoder(), TestConstants.errorMargin) != 0) {
            m_status[0]=true;
        }
        if (MathUtil.applyDeadband(m_pos[1]-m_swerve.getBackLeftDriveEncoder(), TestConstants.errorMargin) != 0) {
            m_status[1]=true;
        }
        if (MathUtil.applyDeadband(m_pos[2]-m_swerve.getFrontRightDriveEncoder(), TestConstants.errorMargin) != 0) {
            m_status[2]=true;
        }
        if (MathUtil.applyDeadband(m_pos[3]-m_swerve.getBackRightDriveEncoder(), TestConstants.errorMargin) != 0) {
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
        m_swerve.setDriveStatus(m_status);
        m_swerve.stopSystem();
    }
}