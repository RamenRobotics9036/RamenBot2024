package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.IntakeSystem;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.ArmConstants;

public class EncoderTestCommand extends CommandBase {

    public boolean isWorking;
    public ArmSystem m_armSystem;

    public Timer m_timer;

    public EncoderTestCommand(ArmSystem armSystem) {
        m_timer = new Timer();

        m_armSystem = armSystem;
        addRequirements(m_armSystem);
    }

    @Override
    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_armSystem.getArmAngle() == 0 && m_timer.get() >= 3) {
            isWorking = false;
        }
    }

    @Override
    public boolean isFinished() {
        if (isWorking == false) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopSystem();
    }

}
