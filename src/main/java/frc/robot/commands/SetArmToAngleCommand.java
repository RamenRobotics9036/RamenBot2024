package frc.robot.commands;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmAndIntakeSystem;

public class SetArmToAngleCommand extends CommandBase {
    private double m_desiredAngle;
    private ArmAndIntakeSystem m_armSystem;
    private DutyCycleEncoder m_ArmEncoder;

    public SetArmToAngleCommand(ArmAndIntakeSystem armSystem,
            double desiredAngle,
            DutyCycleEncoder ArmEncoder) {
        m_desiredAngle = desiredAngle;
        m_armSystem = armSystem;
        m_ArmEncoder = ArmEncoder;
        addRequirements(m_armSystem);
    }

    @Override
    public void initialize() {

        super.initialize();
    }

    @Override
    public void execute() {
        m_armSystem.setArmSpeed(m_desiredAngle);

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        if (m_armSystem.getArmangle() != m_desiredAngle) {
            return false;
        }
        return true;

    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_armSystem.stopSystem();
    }

}
