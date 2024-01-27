package frc.robot.util;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;

import com.opencsv.CSVWriter;

public class CsvWriter {

    public void Datawrite() throws IOException {

        String[] entries = {
                "book", "coin", "pencil", "cup"
        };
        String fileName = "items.csv";

        try (var fos = new FileOutputStream(fileName);
                var osw = new OutputStreamWriter(fos, StandardCharsets.UTF_8);
                var writer = new CSVWriter(osw)) {

            writer.writeNext(entries);
        }
    }
}
