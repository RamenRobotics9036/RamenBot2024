package frc.robot.commands.TestCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestConstants;
import frc.robot.subsystems.ArmSystem;

public class ArmMotorTestCommand extends Command{
    public ArmSystem m_arm;

    public Timer m_timer = new Timer();

    public double m_pos;
    public boolean m_status;

    public ArmMotorTestCommand(ArmSystem arm){
        m_arm = arm;
        addRequirements(m_arm);

        m_pos = m_arm.getArmAngleRadians();
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        m_arm.setArmSpeed(TestConstants.testSpeed);

        if (MathUtil.applyDeadband(m_pos-m_arm.getArmAngleRadians(), TestConstants.errorMargin) != 0) {
            m_status=true;
        }
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= 0.5){
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setStatus(m_status);
        m_arm.stopSystem();
    }
}