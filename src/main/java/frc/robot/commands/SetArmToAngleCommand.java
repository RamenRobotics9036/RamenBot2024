package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CommandsConstants.SetArmConstants;
import frc.robot.subsystems.ArmAndIntakeSystem;

public class SetArmToAngleCommand extends CommandBase {
    private double m_desiredAngle;
    private ArmAndIntakeSystem m_armSystem;
    private PIDController m_pid = new PIDController(SetArmConstants.PID_P, SetArmConstants.PID_I,
            SetArmConstants.PID_D);

    public SetArmToAngleCommand(ArmAndIntakeSystem armSystem, double desiredAngle) {
        m_desiredAngle = desiredAngle;
        m_armSystem = armSystem;
        addRequirements(m_armSystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = m_pid.calculate(m_armSystem.getArmAngle(), m_desiredAngle);
        speed = MathUtil.clamp(speed, -SetArmConstants.percentPower, SetArmConstants.percentPower);
        m_armSystem.setArmSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        if (MathUtil.applyDeadband(m_armSystem.getArmAngle() - m_desiredAngle,
                SetArmConstants.errorMargin) == 0) {
            return true;
        }
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopSystem();
    }

}
