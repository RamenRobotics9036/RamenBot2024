package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;

public class DriveRotCommand extends DriveAxisCommand {
    public DriveRotCommand(SwerveDriveSystem swerveSystem, double rotspeed, double maxTime) {
        super(swerveSystem, 0, 0, rotspeed, maxTime);
    }
}
