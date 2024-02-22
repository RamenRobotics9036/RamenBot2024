package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HookSystem;
import frc.robot.util.AppliedController;

public class DefaultHookCommand extends Command {
    private HookSystem m_hookSystem;
    private AppliedController m_controller;

    public DefaultHookCommand(HookSystem hookSystem, AppliedController controller) {
        m_hookSystem = hookSystem;
        m_controller = controller;
        addRequirements(m_hookSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = m_controller.getRightTriggerAxis();
        if (speed == 0) {
            speed = -m_controller.getLeftTriggerAxis();
        }
        m_hookSystem.setHookSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hookSystem.stopSystem();
    }
}
