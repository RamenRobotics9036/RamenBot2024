package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDSystem;

public class LEDResetCommand extends Command {
    private LEDSystem m_LEDSystem;
    // private PIDController m_pid;

    public LEDResetCommand(LEDSystem LEDSystem) {
        m_LEDSystem = LEDSystem;
    }

    @Override
    public void initialize() {
        m_LEDSystem.resetLED();
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
        m_LEDSystem.stopSystem();
    }

}
