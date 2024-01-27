package frc.robot.commands;

import java.io.FileOutputStream;
import java.io.OutputStreamWriter;

import com.opencsv.CSVWriter;
import java.nio.charset.StandardCharsets;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteYCommand extends DriveXCommand {
    private CSVWriter m_csvWriter;
    private double m_timeOffset;

    public WriteYCommand(SwerveDriveSystem swerveSystem,
            CSVWriter csvWriter,
            double speed,
            double maxSpeed,
            double timeOffset) {
        super(swerveSystem, speed, maxSpeed);
        m_csvWriter = csvWriter;
        m_timeOffset = timeOffset;
        try {
            FileOutputStream fileWriter = new FileOutputStream("YResults.csv");
            OutputStreamWriter outputWriter = new OutputStreamWriter(fileWriter,
                    StandardCharsets.UTF_8);
            m_csvWriter = new CSVWriter(outputWriter);
            m_csvWriter.writeNext(new String[] {
                    "time,", "input,", "output,"
            });
        }
        catch (Exception e) {
            cancel();
        }
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        m_csvWriter.writeNext(new String[] {
                String.valueOf(super.m_timer.get() + m_timeOffset) + ",",
                String.valueOf(super.m_speed) + ",",
                String.valueOf(super.m_swerveSystem.getyPosition()) + ","
        });
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
