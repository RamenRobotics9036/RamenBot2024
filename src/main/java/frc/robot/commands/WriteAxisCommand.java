package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveDriveSystem;

public class WriteAxisCommand extends DriveAxisCommand {
    protected double m_timeOffset;

    public WriteAxisCommand(SwerveDriveSystem swerveSystem,
            double xspeed,
            double yspeed,
            double rotspeed,
            double maxTime,
            double timeOffset) {
        super(swerveSystem, xspeed, yspeed, rotspeed, maxTime);
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putNumber("Time", super.m_timer.get() + m_timeOffset);
        SmartDashboard.putNumber("X Speed", super.m_xspeed);
        SmartDashboard.putNumber("Y Speed", super.m_xspeed);
        SmartDashboard.putNumber("Rot Speed", super.m_xspeed);

        SmartDashboard.putNumber("X Output", super.m_swerveSystem.getxPosition());
        SmartDashboard.putNumber("Y Output", super.m_swerveSystem.getyPosition());
        SmartDashboard.putNumber("Rot Output", super.m_swerveSystem.getAnglePosition());
    }
}
