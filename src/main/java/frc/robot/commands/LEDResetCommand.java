package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSystem;

@SuppressWarnings("AbbreviationAsWordInNameCheck")
public class LEDResetCommand extends Command {
    private LEDSystem m_ledSystem;
    // private PIDController m_pid;

    public LEDResetCommand(LEDSystem ledSystem) {
        m_ledSystem = ledSystem;
    }

    @Override
    public void initialize() {
        m_ledSystem.resetLED();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        m_ledSystem.stopSystem();
    }

}
