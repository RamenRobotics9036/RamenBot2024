package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.util.AppliedController;

public class ArmDefaultCommand extends CommandBase {
    // Goal is to get joystick input and turn that into what we want to set the arm to
    private XboxController m_controller;
    private ArmSystem m_armSystem;

    public ArmDefaultCommand(ArmSystem armSystem, AppliedController controller) {
        m_armSystem = armSystem;
        m_controller = controller;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (m_controller.getLeftTriggerAxis() >= OperatorConstants.controllerDeadbandPercent) {
            if (m_controller.getLeftY() >= OperatorConstants.controllerDeadbandPercent) {
                m_armSystem.setArmSpeed(ArmConstants.armSpeedFast);
            }
            else if (m_controller.getLeftY() <= -OperatorConstants.controllerDeadbandPercent) {
                m_armSystem.setArmSpeed(-ArmConstants.armSpeedFast);
            }
        }
        else {
            if (m_controller.getLeftY() >= OperatorConstants.controllerDeadbandPercent) {
                m_armSystem.setArmSpeed(m_controller.getLeftY());
            }
            else if (m_controller.getLeftY() <= -OperatorConstants.controllerDeadbandPercent) {
                m_armSystem.setArmSpeed(-m_controller.getLeftY());
            }
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
