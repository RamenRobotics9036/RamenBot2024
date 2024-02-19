package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HookConstants;
import frc.robot.subsystems.HookSystem;
import frc.robot.util.AppliedController;

public class HookDefaultCommand extends Command {

    private HookSystem m_hook;
    private AppliedController m_controller;
    private boolean m_up;


    public HookDefaultCommand(HookSystem hook, AppliedController controller) {
        m_hook = hook;
        m_controller = controller;
        addRequirements(m_hook);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_controller.getLeftBumper()){
            m_up = true;
        }
        else if (m_controller.getLeftBumper()){
            m_up = false;
        }
        if (m_up){
            if (m_hook.getLeadEncoderValue() <= HookConstants.maxHeight){
                m_hook.setHookSpeed(HookConstants.HookSpeed);
            } else{
                m_hook.setHookSpeed(0);
            }
        } else {
            if (m_hook.getLeadEncoderValue() >= HookConstants.minHeight){
                m_hook.setHookSpeed(-HookConstants.HookSpeed);
            } else{
                m_hook.setHookSpeed(0);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hook.stopSystem();
    }

}
