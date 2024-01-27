package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;

public class DriveXCommand extends DriveAxisCommand {
    public DriveXCommand(SwerveDriveSystem swerveSystem, double xspeed, double maxTime) {
        super(swerveSystem, xspeed, 0, 0, maxTime);
    }
}
