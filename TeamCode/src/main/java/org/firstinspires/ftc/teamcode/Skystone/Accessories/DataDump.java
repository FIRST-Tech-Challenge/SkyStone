package org.firstinspires.ftc.teamcode.Skystone.Accessories;

import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;

@Deprecated
public class DataDump {
    public static void dump(Robot robot, String directoryName){
//        writeToFile("" + directoryName, "odometryModule.txt", robot.odometryModule.odometryData.toString());
//        writeToFile("" + directoryName, "pathModule.txt", robot.pathModule.pathData.toString());
//        writeToFile("" + directoryName, "driveModule.txt", robot.driveModule.driveData.toString());
//        writeToFile("" + directoryName, "intakeModule.txt", robot.intakeModule.intakeData.toString());
//        writeToFile("" + directoryName, "linkageModule.txt", robot.outtakeModule.linkageModule.linkageData.toString());
//        writeToFile("" + directoryName, "clawModule.txt", robot.outtakeModule.clawModule.clawData.toString());
//        writeToFile("" + directoryName, "spoolModule.txt", robot.outtakeModule.spoolModule.spoolData.toString());
//        writeToFile("" + directoryName, "foundationMoverModule.txt", robot.foundationMoverModule.foundationMoverData.toString());
    }

    private static void writeToFile(String directoryName, String fileName, String data) {
        File captureDirectory = new File(AppUtil.ROBOT_DATA_DIR, "/" + directoryName + "/");
        if (!captureDirectory.exists()) {
            boolean isFileCreated = captureDirectory.mkdirs();
            Log.d("DumpToFile", " " + isFileCreated);
        }
        Log.d("DumpToFile", " hey ");
        File file = new File(captureDirectory, fileName);
        try {
            FileOutputStream outputStream = new FileOutputStream(file);
            OutputStreamWriter writer = new OutputStreamWriter(outputStream);
            try {
                writer.write(data);
                writer.flush();
                Log.d("DumpToFile", data);
            } finally {
                outputStream.close();
                Log.d("DumpToFile", file.getAbsolutePath());
            }
        } catch (IOException e) {
            RobotLog.ee("TAG", e, "exception in captureFrameToFile()");
        }
    }
}
