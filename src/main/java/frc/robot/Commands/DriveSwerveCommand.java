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
        double xSpeed = m_controller.getLeftX();
        double ySpeed = m_controller.getLeftY();
        double rot = m_controller.getRightX();
        // m_swerveDrive.drive(0, 0.2, 0);

        // $TODO - Inverting y on joystick is a hack right now!
        m_swerveDrive.drive(xSpeed, -ySpeed, rot);
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
