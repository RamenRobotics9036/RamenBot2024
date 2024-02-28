package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandsConstants.SetArmConstants;
import frc.robot.subsystems.ArmSystem;

public class SetArmToAngleCommand extends Command {
    private double m_desiredAngle;
    private ArmSystem m_armSystem;
    private Timer m_timer;
    // private PIDController m_pid;

    public SetArmToAngleCommand(ArmSystem armSystem, double desiredAngle) {
        m_desiredAngle = desiredAngle;
        m_armSystem = armSystem;
        addRequirements(m_armSystem);
    }

    @Override
    public void initialize() {
        /*
         * Couldnt get PID tuned.
         */
        // m_pid = new PIDController(SetArmConstants.PID_P, SetArmConstants.PID_I,
        // SetArmConstants.PID_D);
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        double speed = m_armSystem.getArmAngleRadians() - m_desiredAngle;
        m_armSystem.setArmSpeed(speed * 5);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= SetArmConstants.maxTime) {
            return true;
        }
        if (MathUtil.applyDeadband(
                m_armSystem.getArmAngleRadians() - m_desiredAngle,
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
