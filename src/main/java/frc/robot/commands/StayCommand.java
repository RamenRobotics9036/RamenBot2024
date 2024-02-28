package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSystem;

public class StayCommand extends Command {
    private SwerveDriveSystem m_swerveDrive;

    public StayCommand(SwerveDriveSystem swerveDrive) {
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_swerveDrive.drive(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
