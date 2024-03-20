package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSystem;

public class PullBackCommand extends Command {
    private IntakeSystem m_intakeSystem;
    private Timer m_timer = new Timer();

    public PullBackCommand(IntakeSystem intake) {
        m_intakeSystem = intake;
        addRequirements(m_intakeSystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_intakeSystem.setIntakeSpeed(-IntakeConstants.intakeSpeed);
    }

    @Override
    public boolean isFinished() {
        // 1 is a dummy value. Please change it or divide speed by something later.
        if (m_timer.get() >= 1) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
