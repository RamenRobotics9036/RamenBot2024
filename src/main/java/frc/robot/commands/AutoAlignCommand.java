package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.subsystems.VisionSystem;

public class AutoAlignCommand extends Command {
    private VisionSystem m_visionSystem;
    private SwerveDriveSystem m_swerveDrive;
    private Timer m_timer;
    private double m_initialAngle;

    public AutoAlignCommand(VisionSystem visionSystem, SwerveDriveSystem swerveDrive) {
        m_visionSystem = visionSystem;
        m_swerveDrive = swerveDrive;
        addRequirements(m_visionSystem, m_swerveDrive);

        m_initialAngle = m_swerveDrive.getRotation2d().getDegrees();
    }

    @Override
    public void initialize() {
        if (!m_visionSystem.getIsDetecting()) {
            cancel();
        }
        m_timer = new Timer();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_swerveDrive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() >= 4) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_visionSystem.stopSystem();
        m_swerveDrive.stopSystem();
    }
}
