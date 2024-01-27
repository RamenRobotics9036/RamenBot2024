package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteRotCommand extends WriteAxisCommand {
    public WriteRotCommand(SwerveDriveSystem swerveSystem,
            double rotspeed,
            double timeOffset,
            double maxTime) {
        super(swerveSystem, 0, 0, rotspeed, maxTime, timeOffset, "rotSpreadsheet.csv");
    }

    @Override
    public void execute() {
        super.execute();
        try {
            super.m_csvWriter.writeNext(new String[] {
                    String.valueOf(super.m_timer.get() + super.m_timeOffset) + ",",
                    String.valueOf(super.m_rotspeed) + ",",
                    String.valueOf(super.m_swerveSystem.getAnglePosition()) + ","
            });
        }
        catch (Exception e) {
            cancel();
        }
    }
}
