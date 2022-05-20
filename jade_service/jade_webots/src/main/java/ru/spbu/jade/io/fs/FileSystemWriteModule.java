package ru.spbu.jade.io.fs;

import ru.spbu.jade.io.WriteModule;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class FileSystemWriteModule implements WriteModule {

    private final File outFile;

    public FileSystemWriteModule(File outFile) {
        this.outFile = outFile;
    }

    @Override
    public void writeValue(String value) {
        try (FileWriter writer = new FileWriter(outFile, false)) {
            writer.write(value);
            writer.flush();
        } catch (IOException e) {
            throw new RuntimeException("Can't write new azimuth to file: " + outFile);
        }
    }
}
