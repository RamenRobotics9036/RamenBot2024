package frc.robot.commands.TestCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.VisionSystem;

public class LimelightGetPos extends Command {

    VisionSystem m_lime;

    public LimelightGetPos(VisionSystem lime) {
        m_lime = lime;
        addRequirements(m_lime);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        Shuffleboard.getTab("LimeTest")
                .addDoubleArray("Coords", () -> LimelightHelpers.getBotPose(getName()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
