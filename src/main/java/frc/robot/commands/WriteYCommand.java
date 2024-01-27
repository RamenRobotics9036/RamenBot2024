package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteYCommand extends WriteAxisCommand {
    public WriteYCommand(SwerveDriveSystem swerveSystem, double yspeed, double maxTime) {
        super(swerveSystem, 0, yspeed, 0, maxTime);
    }

    @Override
    public void execute() {
        super.execute();
    }
}
