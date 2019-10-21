package org.firstinspires.ftc.teamcode.roboticslibrary;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.util.Arrays;

/**
 * Created by Nirzvi on 2015-05-23.
 */
public class DataWriter extends FileOutputStream {

    public DataWriter(String fileName, boolean overWrite) throws FileNotFoundException {

        super(new File(AppUtil.getInstance().getActivity().getExternalFilesDir(null).getAbsolutePath() + "/" + fileName.replace(".txt", "") + ".txt"), !overWrite);

    }

    public DataWriter(String filePath, String fileName, boolean overWrite) throws FileNotFoundException {

        super(new File(filePath + fileName + ".txt"), !overWrite);

    }

    public void write (String data) {

        try {
            write(("" + data).getBytes());
        } catch (Exception e) {

        }
    }

    public void write (int data) {
        write("" + data);
    }//write

    public void write (double data) {
        write("" + data);
    }//write

    public void write (float data) {
        write("" + data);
    }//write

    public void write (long data) {
        write("" + data);
    }//write

    public void write (short data) {
        write("" + data);
    }//write

    public void write (byte data) {
        write("" + data);
    }//write

    public void write (char data) {
        write("" + data);
    }//write

    public void write (boolean data) {
        write("" + data);
    }//write

    public void write (Object data) {
        write(data.toString());
    }//write

    public void write (int[] data) {
        write(Arrays.toString(data));
    }//write

    public void write (double[] data) {
        write(Arrays.toString(data));
    }//write

    public void write (float[] data) {
        write(Arrays.toString(data));
    }//write

    public void write (long[] data) {
        write(Arrays.toString(data));
    }//write

    public void write (short[] data) {
        write(Arrays.toString(data));
    }//write

    public void write (char[] data) {
        write(Arrays.toString(data));
    }//write

    public void write (boolean[] data) {
        write(Arrays.toString(data));
    }//write

    public void write (Object[] data) {
        write(Arrays.toString(data));
    }//write

    public void closeWriter () {
        try {
            close();
        } catch (Exception e) {
            e.printStackTrace();
        }//catch
    }//closeWriter

}//DataWriter
