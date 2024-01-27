package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSystem;

public class DriveYCommand extends DriveAxisCommand {
    public DriveYCommand(SwerveDriveSystem swerveSystem, double yspeed, double maxTime) {
        super(swerveSystem, 0, yspeed, 0, maxTime);
    }
}
