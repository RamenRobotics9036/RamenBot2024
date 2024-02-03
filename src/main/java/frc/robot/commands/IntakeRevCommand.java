package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RevConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.CommandsConstants.IntakeReleaseConstants;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;

public class IntakeRevCommand extends CommandBase {
    public ShooterSystem m_shooter;
    public IntakeSystem m_intake;
    public Timer m_timer;

    public IntakeRevCommand(IntakeSystem intake, ShooterSystem outtake) {
        m_intake = intake;
        m_shooter = outtake;

        m_timer = new Timer();
        addRequirements(m_intake, m_shooter);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_timer.get() >= RevConstants.revTime) {
            m_intake.setIntakeSpeed(IntakeConstants.intakeSpeed);
        }
        m_shooter.setShootSpeed(ShooterConstants.shooterSpeed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= IntakeReleaseConstants.maxTime) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopSystem();
    }
}
