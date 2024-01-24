package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ArmSystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class ArmDefaultCommand extends CommandBase {
    // Goal is to get joystick input and turn that into what we want to set the arm to
    private XboxController m_Controller;
    private ArmSystem m_arm = new ArmSystem();

    public ArmDefaultCommand(ArmSystem arm) {
        m_arm = arm;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        Shuffleboard.getTab("Joystick Value").add("Joystick Value: ", m_Controller.getLeftY());
        if (m_Controller.getLeftTriggerAxis() >= OperatorConstants.controllerDeadbandPercent) {
            if (m_Controller.getLeftY() >= OperatorConstants.controllerDeadbandPercent) {
                m_arm.setArmSpeed(ArmConstants.armSpeedFast);
            }
            else if (m_Controller.getLeftY() <= -OperatorConstants.controllerDeadbandPercent) {
                m_arm.setArmSpeed(-ArmConstants.armSpeedFast);
            }
        }
        else {
            if (m_Controller.getLeftY() >= OperatorConstants.controllerDeadbandPercent) {
                m_arm.setArmSpeed(m_Controller.getLeftY());
            }
            else if (m_Controller.getLeftY() <= -OperatorConstants.controllerDeadbandPercent) {
                m_arm.setArmSpeed(-m_Controller.getLeftY());
            }
        }

    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        m_arm.stopSystem();
    }
}
