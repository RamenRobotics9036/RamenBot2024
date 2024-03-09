package frc.robot.commands.testcommand;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.SwerveDriveSystem;

public class SwerveMoveMotorTestCommand extends Command {
    public SwerveDriveSystem m_swerve;
    public Timer m_timer = new Timer();
    public boolean[] m_worked = new boolean[4];

    public SwerveMoveMotorTestCommand(SwerveDriveSystem swerve) {
        m_swerve = swerve;
        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_swerve.drive(TestConstants.testSpeed, TestConstants.testSpeed, 0);
        if (m_swerve.getFrontLeftDriveVelocity() >= 0.1) {
            m_worked[0] = true;
        }
        if (m_swerve.getBackLeftDriveVelocity() >= 0.1) {
            m_worked[1] = true;
        }
        if (m_swerve.getFrontRightDriveVelocity() >= 0.1) {
            m_worked[2] = true;
        }
        if (m_swerve.getBackRightDriveVelocity() >= 0.1) {
            m_worked[3] = true;
        }
        m_swerve.setStatus(m_worked);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= TestConstants.testTime) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stopSystem();
    }
}
