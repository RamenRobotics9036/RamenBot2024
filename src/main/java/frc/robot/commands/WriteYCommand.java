package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteYCommand extends WriteAxisCommand {
    public WriteYCommand(SwerveDriveSystem swerveSystem,
            double yspeed,
            double timeOffset,
            double maxTime) {
        super(swerveSystem, 0, yspeed, 0, maxTime, timeOffset);
    }

    @Override
    public void execute() {
        super.execute();
    }
}
