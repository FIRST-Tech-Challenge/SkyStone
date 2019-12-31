package org.firstinspires.ftc.teamcode.All;

import android.util.Log;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

public class DriveConstant {
    public static double WHEEL_ENCODER_COUNTS_PER_REVOLUTION = 383.6;     //@TODO Test and find the number of encoder counts for 1 revolution
    public static double ODOMETRY_ENCODER_COUNTS_PER_REVOLUTION = 1400.0;

    public static double ODOMETRY_RAD = 2.9;    //in cm
    public static double MECANUM_RAD = 2.0;     //in in

    public static double MOTOR_GEAR_RATIO = 2.0 / 1.0;     //@TODO Change the gear ratios [Encoder gear / Wheel gear] 24/40?
    public static double ODOMETRY_GEAR_RATIO = 1.0 / 1.0;


    public static void writeFile(String filePath, String data) {
        try {
            File folder = new File(filePath.substring(0,filePath.lastIndexOf("/") + 1));
            folder.mkdirs();
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(new FileOutputStream(new File(filePath), true));
            outputStreamWriter.write(data);
            outputStreamWriter.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    public static void writeSerializedObject(String filePath, Object data) {
        try {
            File folder = new File(filePath.substring(0, filePath.lastIndexOf("/") + 1));
            folder.mkdirs();
            ObjectOutputStream outputStreamWriter = new ObjectOutputStream(new FileOutputStream(new File(filePath), true));
            outputStreamWriter.writeObject(data);
            outputStreamWriter.close();
        }
        catch (IOException e) {
            Log.e("Exception", "File write failed: " + e.toString());
        }
    }

    public static Object getSerializedObject(String filePath){
        Object output = null;
        try {
            File folder = new File(filePath.substring(0, filePath.lastIndexOf("/") + 1));
            folder.mkdirs();
            ObjectInputStream outputStreamReader = new ObjectInputStream(new FileInputStream(new File(filePath)));
            output = outputStreamReader.readObject();
            outputStreamReader.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return output;
    }
}
