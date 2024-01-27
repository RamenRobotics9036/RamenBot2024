package frc.robot.commands;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;

import com.opencsv.CSVWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CSVConstants;
import frc.robot.Constants.OperatorConstants;

public class CsvWriterCommand extends CommandBase {
    private Timer m_TIMER = new Timer();
    private String Time;
    private String XboxInput;
    private XboxController m_CONTROLLER = new XboxController(OperatorConstants.driveControllerPort);

    public void Datawrite() throws IOException {

        //

        String[] entries = {
                // Time, XBOXCONTROLERINPUT
                Time, XboxInput

        };

        String fileName = "items(1).csv";

        try (var fos = new FileOutputStream(fileName);
                var osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
                var writer = new CSVWriter(osw)) {

            writer.writeNext(entries);
        }
    }

    @Override
    public void initialize() {
        m_TIMER.start();
    }

    @Override
    public void execute() {
        Time = "Time: " + m_TIMER.get();
        XboxInput = "Left Stick X Input:" + m_CONTROLLER.getLeftX();
        try {
            Datawrite();
        }
        catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Override
    public boolean isFinished() {
        if (m_TIMER.get() >= CSVConstants.CSVLimit) {
            return true;
        }
        return false;

    }

    @Override
    public void end(boolean interrupted) {
        m_TIMER.stop();
    }
}
