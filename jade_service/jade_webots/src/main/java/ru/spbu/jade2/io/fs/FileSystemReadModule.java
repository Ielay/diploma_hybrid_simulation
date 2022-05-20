package ru.spbu.jade2.io.fs;

import ru.spbu.jade2.io.ReadModule;

import java.io.File;
import java.util.Scanner;

public class FileSystemReadModule implements ReadModule {

    private final File inFile;

    public FileSystemReadModule(File inFile) {
        this.inFile = inFile;
    }

    @Override
    public String readValue() {
        StringBuilder sb = new StringBuilder();
        try (Scanner scanner = new Scanner(inFile)) {
            while (scanner.hasNextLine()) {
                sb.append(scanner.nextLine());
                sb.append("\n");
            }
        } catch (Throwable e) {
            throw new RuntimeException("Error reading value from file " + inFile.toString(), e);
        }
        return sb.toString();
    }
}
