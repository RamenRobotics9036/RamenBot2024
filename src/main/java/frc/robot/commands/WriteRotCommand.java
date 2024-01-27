package frc.robot.commands;

import java.io.FileOutputStream;
import java.io.OutputStreamWriter;

import com.opencsv.CSVWriter;
import java.nio.charset.StandardCharsets;

import frc.robot.subsystems.SwerveDriveSystem;

public class WriteRotCommand extends DriveXCommand {
    private CSVWriter m_csvWriter;
    private double m_timeOffset;

    public WriteRotCommand(SwerveDriveSystem swerveSystem,
            double speed,
            double maxTime,
            double timeOffset) {
        super(swerveSystem, speed, maxTime);
        m_timeOffset = timeOffset;
        try {
            FileOutputStream fileWriter = new FileOutputStream("rotResults.csv");
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
                String.valueOf(super.m_swerveSystem.getAnglePosition()) + ","
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
