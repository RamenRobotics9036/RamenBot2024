package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CommandsConstants.SetArmConstants;
import frc.robot.subsystems.ArmSystem;

public class SetArmToAngleCommand extends CommandBase {
    private double m_desiredAngle;
    private ArmSystem m_armSystem;
    private PIDController m_pid = new PIDController(SetArmConstants.PID_P, SetArmConstants.PID_I,
            SetArmConstants.PID_D);

    public SetArmToAngleCommand(ArmSystem armSystem, double desiredAngle) {
        m_desiredAngle = desiredAngle;
        m_armSystem = armSystem;
        addRequirements(m_armSystem);
        m_pid.setTolerance(SetArmConstants.errorMarginPosition,
                SetArmConstants.errorMarginVelocity);
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
        if (m_pid.atSetpoint()) {
            return true;
        }
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopSystem();
    }

}
