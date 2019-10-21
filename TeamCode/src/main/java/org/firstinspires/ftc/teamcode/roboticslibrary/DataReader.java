package org.firstinspires.ftc.teamcode.roboticslibrary;

import android.content.Context;

import org.firstinspires.ftc.teamcode.RC;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

/**
 * Created by FIXIT on 2015-05-23.
 */
public class DataReader extends BufferedReader {

    public int numLines;

    public DataReader(String fileName) throws IOException {
            super(new FileReader(new File(RC.c().getExternalFilesDir(null).getAbsolutePath() + "/" + fileName.replace(".txt", "") + ".txt")));
            mark(Short.MAX_VALUE);

            while (readLine() != null) {
                numLines++;
            }
            reset();
    }

    public String readFile() {

        String data = "";

        try {
            for (int i = 0; i < numLines; i++) {
                data += readLine() + "\n";
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return data;
    }

    public String[] readLines() {

        String[] data = new String[numLines];

        try {
            for (int i = 0; i < numLines; i++) {
                data[i] = readLine();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        return data;
    }

}
