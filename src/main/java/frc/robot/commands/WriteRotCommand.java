package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteRotCommand extends WriteAxisCommand {
    public WriteRotCommand(SwerveDriveSystem swerveSystem,
            double rotspeed,
            double timeOffset,
            double maxTime) {
        super(swerveSystem, 0, 0, rotspeed, maxTime, timeOffset);
    }
}
