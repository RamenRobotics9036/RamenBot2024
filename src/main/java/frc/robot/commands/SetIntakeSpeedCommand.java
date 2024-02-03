package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSystem;

public class SetIntakeSpeedCommand extends CommandBase {
    private IntakeSystem m_intakeSystem;
    private double m_maxTime;
    private double m_speed;
    private Timer m_timer;

    public SetIntakeSpeedCommand(IntakeSystem intakeSystem, double maxTime, double speed) {
        m_intakeSystem = intakeSystem;
        m_maxTime = maxTime;
        m_speed = speed;
        m_timer = new Timer();
        addRequirements(m_intakeSystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_intakeSystem.setIntakeSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= m_maxTime) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSystem.stopSystem();
    }
}
