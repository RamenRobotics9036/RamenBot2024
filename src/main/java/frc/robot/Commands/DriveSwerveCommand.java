package frc.robot.commands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.util.AppliedController;

/**
 * DriveSwerveCommand.
 */
public class DriveSwerveCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private AppliedController m_controller;

    /**
     * Constructor.
     */
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
        if (m_controller.getLeftBumper()) {
            // GYRO FIELD RELATIVE RESET in terms of the right side of the robot (radio side)
            m_swerveDrive.resetGyroFieldRelative();
        }
        else {
            double xspeed = m_controller.getLeftX();
            double yspeed = m_controller.getLeftY();
            double rot = m_controller.getRightX();

            // Turns off field relative while pressed
            m_swerveDrive.setFieldRelative(!m_controller.getRightBumper());

            // $TODO - Inverting y on joystick is a hack right now!
            m_swerveDrive.drive(xspeed, -yspeed, rot);
        }

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
