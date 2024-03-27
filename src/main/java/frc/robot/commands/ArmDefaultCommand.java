package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSystem;
import frc.robot.util.AppliedController;

public class ArmDefaultCommand extends Command {
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
        if (m_controller.getRightBumper()) {
            m_armSystem.setArmSpeed(-m_controller.getRightY());
        }
        else {
            m_armSystem
                    .setArmSpeed(
                            -ArmConstants.armSpeedFast * Math.signum(m_controller.getRightY()));
        }
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
        m_armSystem.stopSystem();
    }
}
