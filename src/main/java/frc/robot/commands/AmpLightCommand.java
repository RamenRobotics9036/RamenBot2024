package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSystem;
import frc.robot.subsystems.LEDSystem;

public class AmpLightCommand extends Command {
    private LEDSystem m_LEDSystem;
    // private PIDController m_pid;

    public AmpLightCommand(LEDSystem LEDSystem) {
        m_LEDSystem = LEDSystem;
    }

    @Override
    public void initialize() {
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
