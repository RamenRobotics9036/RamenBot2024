package frc.robot.commands;

import frc.robot.Constants.TimeConstants;
import frc.robot.subsystems.SwerveDriveSystem;

public class WriteAxisCommand extends DriveAxisCommand {
    protected double m_timeOffset;

    public WriteAxisCommand(SwerveDriveSystem swerveSystem,
            double xspeed,
            double yspeed,
            double rotspeed,
            double maxTime) {
        super(swerveSystem, xspeed, yspeed, rotspeed, maxTime);
    }

    @Override
    public void initialize() {
        super.initialize();
        TimeConstants.logTimer.start();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        TimeConstants.logTimer.stop();
    }
}
