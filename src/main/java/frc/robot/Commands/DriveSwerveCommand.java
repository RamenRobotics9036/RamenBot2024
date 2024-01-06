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
        if (m_controller.getLeftBumper()){
            m_swerveDrive.resetGyroFieldRelative(); //GYRO FIELD RELATIVE RESET in terms of the right side of the robot (radio side)
        } else {
            double xSpeed = m_controller.getLeftX();
            double ySpeed = m_controller.getLeftY();
            double rot = m_controller.getRightX();

            m_swerveDrive.setFieldRelative(!m_controller.getRightBumper()); // Turns off field relative while pressed 
            
            // $TODO - Inverting y on joystick is a hack right now!
            m_swerveDrive.drive(xSpeed, -ySpeed, rot);
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
