package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteXCommand extends WriteAxisCommand {
    public WriteXCommand(SwerveDriveSystem swerveSystem,
            double xspeed,
            double timeOffset,
            double maxTime) {
        super(swerveSystem, xspeed, 0, 0, maxTime, timeOffset, "xSpreadsheet.csv");
    }

    @Override
    public void execute() {
        super.execute();
        try {
            super.m_csvWriter.writeNext(new String[] {
                    String.valueOf(super.m_timer.get() + super.m_timeOffset) + ",",
                    String.valueOf(super.m_xspeed) + ",",
                    String.valueOf(super.m_swerveSystem.getxPosition()) + ","
            });
        }
        catch (Exception e) {
            cancel();
        }
    }
}
