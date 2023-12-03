package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.AppliedController;

public class DriveSwerveCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private AppliedController m_controller;

    public DriveSwerveCommand(SwerveDriveSystem swerveDrive, AppliedController controller) {
        m_swerveDrive = swerveDrive;
        m_controller = controller;
        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double forwardBackSpeed = m_controller.getLeftY();
        double leftRightSpeed = m_controller.getLeftX();
        double rot = m_controller.getRightX();
        m_swerveDrive.drive(forwardBackSpeed, leftRightSpeed, rot);
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
