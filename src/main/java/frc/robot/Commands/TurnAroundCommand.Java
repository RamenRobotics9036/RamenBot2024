package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.SwerveDriveSystem;
import frc.robot.Util.AppliedController;

public class TurnAroundCommand extends CommandBase {
    private SwerveDriveSystem m_swerveDrive;
    private double m_startAngle;
    private double m_endAngle;

    public DriveSwerveCommand(SwerveDriveSystem swerveDrive) {
        m_swerveDrive = swerveDrive;
        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_startAngle = m_swerveDrive.getYO();
        m_endAngle = (m_startAngle + 180);
        if (m_endAngle > 360){
            m_endAngle -= 360;
        }
    }

    @Override
    public void execute() {
        double currentAngle = m_swerveDrive.getYO();
        
        double rot = 1.0 * (currentAngle - m_endAngle);
        m_swerveDrive.drive(0, 0, rot);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = m_swerveDrive.getYO();
        double difference = Math.abs(m_endAngle-currentAngle);

        if (difference < 1){
            return true;
        }
        else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
