package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;
import frc.robot.util.AppliedController;

/**
 * DriveSwerveCommand.
 */
public class DriveSwerveCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private AppliedController m_controller;
    private Timer m_timer = new Timer();
    private boolean[] m_working = new boolean[4];
    private double[] m_oldPositions = new double[4];
    private double[] m_changes = {
            0, 0, 0, 0
    };

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
        m_oldPositions[0] = m_swerveDrive.getFrontLeftDriveEncoder();
        m_oldPositions[1] = m_swerveDrive.getBackLeftDriveEncoder();
        m_oldPositions[2] = m_swerveDrive.getFrontRightDriveEncoder();
        m_oldPositions[3] = m_swerveDrive.getBackRightDriveEncoder();

        m_timer.start();
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

            // Quick test I made to make sure encoders are working. Hopefully it works.
            if (xspeed != 0 || yspeed != 0 || rot != 0) {

                if (m_swerveDrive.getFrontLeftDriveEncoder() >= m_oldPositions[0]) {
                    m_changes[0] += m_swerveDrive.getFrontLeftDriveEncoder() - m_oldPositions[0];
                    m_oldPositions[0] = m_swerveDrive.getFrontRightDriveEncoder();
                }
                if (m_swerveDrive.getBackLeftDriveEncoder() >= m_oldPositions[1]) {
                    m_changes[1] += m_swerveDrive.getBackLeftDriveEncoder() - m_oldPositions[1];
                    m_oldPositions[1] = m_swerveDrive.getFrontRightDriveEncoder();
                }
                if (m_swerveDrive.getFrontRightDriveEncoder() >= m_oldPositions[2]) {
                    m_changes[2] += m_swerveDrive.getFrontRightDriveEncoder() - m_oldPositions[2];
                    m_oldPositions[2] = m_swerveDrive.getFrontRightDriveEncoder();
                }
                if (m_swerveDrive.getBackRightDriveEncoder() >= m_oldPositions[3]) {
                    m_changes[3] += m_swerveDrive.getBackRightDriveEncoder() - m_oldPositions[3];
                    m_oldPositions[3] = m_swerveDrive.getFrontRightDriveEncoder();
                }
                m_timer.start();

                if (m_timer.get() % 1 == 0) {
                    if (m_changes[0] > 1) {
                        m_working[0] = true;
                    }
                    else {
                        m_working[0] = false;
                    }
                    if (m_changes[1] > 1) {
                        m_working[1] = true;
                    }
                    else {
                        m_working[1] = false;
                    }
                    if (m_changes[2] > 1) {
                        m_working[2] = true;
                    }
                    else {
                        m_working[2] = false;
                    }
                    if (m_changes[3] > 1) {
                        m_working[3] = true;
                    }
                    else {
                        m_working[3] = false;
                    }
                }
                m_swerveDrive.setStatus(m_working);
            }
            else {
                m_timer.stop();
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
