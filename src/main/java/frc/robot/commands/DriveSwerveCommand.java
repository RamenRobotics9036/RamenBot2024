package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.util.AppliedController;

/**
 * DriveSwerveCommand.
 */
public class DriveSwerveCommand extends Command {
    private SwerveDriveSystem m_swerveDrive;
    private AppliedController m_controller;
    private boolean[] m_working = new boolean[4];

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
        if (m_controller.getStartButtonPressed()) {
            // GYRO FIELD RELATIVE RESET in terms of the right side of the robot (radio side)
            m_swerveDrive.resetGyroFieldRelative();
            return;
        }

        double xspeed = m_controller.getLeftX();
        double yspeed = m_controller.getLeftY();
        double rot = m_controller.getRightX();

        // Turns off field relative while pressed
        m_swerveDrive.setFieldRelative(!m_controller.getLeftStickButton());

        // $TODO - Inverting y on joystick is a hack right now!
        m_swerveDrive.drive(xspeed, -yspeed, rot);

        // Quick test I made to make sure encoders are working. Hopefully it works.
        if (xspeed != 0 || yspeed != 0 || rot != 0) {

            if (m_swerveDrive.getFrontLeftDriveVelocity() > 0.1
                    || m_swerveDrive.getFrontLeftDriveVelocity() < -0.1) {
                m_working[0] = true;
            }
            else {
                m_working[0] = false;
            }

            if (m_swerveDrive.getBackLeftDriveVelocity() > 0.1
                    || m_swerveDrive.getBackLeftDriveVelocity() < -0.1) {
                m_working[1] = true;
            }
            else {
                m_working[1] = false;
            }

            if (m_swerveDrive.getFrontRightDriveVelocity() > 0.1
                    || m_swerveDrive.getFrontRightDriveVelocity() < -0.1) {
                m_working[2] = true;
            }
            else {
                m_working[2] = false;
            }

            if (m_swerveDrive.getBackRightDriveVelocity() > 0.1
                    || m_swerveDrive.getBackRightDriveVelocity() < -0.1) {
                m_working[3] = true;
            }
            else {
                m_working[3] = false;
            }

                m_swerveDrive.setDriveStatus(m_working);
            }
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
