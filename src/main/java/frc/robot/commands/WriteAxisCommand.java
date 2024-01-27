package frc.robot.commands;

import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;

import com.opencsv.CSVWriter;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteAxisCommand extends DriveAxisCommand {
    protected CSVWriter m_csvWriter;
    protected double m_timeOffset;

    public WriteAxisCommand(SwerveDriveSystem swerveSystem,
            double xspeed,
            double yspeed,
            double rotspeed,
            double maxTime,
            double timeOffset,
            String name) {
        super(swerveSystem, xspeed, yspeed, rotspeed, maxTime);
        try {
            FileOutputStream fileWriter = new FileOutputStream(name);
            OutputStreamWriter outputWriter = new OutputStreamWriter(fileWriter,
                    StandardCharsets.UTF_8);
            m_timeOffset = timeOffset;
            m_csvWriter = new CSVWriter(outputWriter);
            m_csvWriter.writeNext(new String[] {
                    "time,", "input,", "output,"
            });
        }
        catch (Exception e) {
            cancel();
        }
    }
}
