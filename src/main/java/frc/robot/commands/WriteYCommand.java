package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteYCommand extends WriteAxisCommand {
    public WriteYCommand(SwerveDriveSystem swerveSystem,
            double rotspeed,
            double timeOffset,
            double maxTime) {
        super(swerveSystem, 0, 0, rotspeed, maxTime, timeOffset, "ySpreadsheet.csv");
    }

    @Override
    public void execute() {
        super.execute();
        try {
            super.m_csvWriter.writeNext(new String[] {
                    String.valueOf(super.m_timer.get() + super.m_timeOffset) + ",",
                    String.valueOf(super.m_yspeed) + ",",
                    String.valueOf(super.m_swerveSystem.getyPosition()) + ","
            });
        }
        catch (Exception e) {
            cancel();
        }
    }
}
