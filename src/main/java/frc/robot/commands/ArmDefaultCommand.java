package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.util.AppliedController;

public class ArmDefaultCommand extends CommandBase {
    // Goal is to get joystick input and turn that into what we want to set the arm to
    private AppliedController m_controller;
    private ArmSystem m_armSystem;

    public ArmDefaultCommand(ArmSystem armSystem, AppliedController controller) {
        m_armSystem = armSystem;
        m_controller = controller;
        addRequirements(m_armSystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_controller.getLeftTriggerAxis() != 0) {
            m_armSystem
                    .setArmSpeed(-ArmConstants.armSpeedFast * Math.signum(m_controller.getLeftY()));
        }
        else {
            m_armSystem.setArmSpeed(-m_controller.getLeftY());
        }
    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopSystem();
    }
}
