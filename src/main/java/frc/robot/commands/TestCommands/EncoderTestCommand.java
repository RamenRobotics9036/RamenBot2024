package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSystem;

public class EncoderTestCommand extends Command {

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
        if (m_armSystem.getArmAngleRadians() == 0 && m_timer.get() >= 3) {
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
