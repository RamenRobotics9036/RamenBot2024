package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

public class AutoStopIntakeCommand extends Command {
    private IntakeSystem m_intakeSystem;

    public AutoStopIntakeCommand(IntakeSystem intakeSystem) {
        m_intakeSystem = intakeSystem;
        addRequirements(m_intakeSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        m_intakeSystem.setIntakeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        // $TODO - Remove this
        if (RobotBase.isSimulation()) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSystem.stopSystem();
    }
}
