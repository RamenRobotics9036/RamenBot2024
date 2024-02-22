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
        double leftSpeed = (m_controller.getLeftTriggerAxis() > 0) ? 1
                : (m_controller.getLeftBumper()) ? -1 : 0;
        m_hookSystem.setHookSpeedLeft(leftSpeed);

        double rightSpeed = (m_controller.getRightTriggerAxis() > 0) ? 1
                : (m_controller.getRightBumper()) ? -1 : 0;
        m_hookSystem.setHookSpeedRight(rightSpeed);
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
