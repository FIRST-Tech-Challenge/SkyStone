package org.firstinspires.ftc.teamcode.lib;

import android.content.Context;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

public class HardwareMapper {

    public static boolean writeHardwareMap() {
        RobotConfigFileManager robotConfigFileManager = new RobotConfigFileManager();
        RobotConfigFile file = new RobotConfigFile(robotConfigFileManager, "generated");
        try {
            File res = AppUtil.getDefContext().getDir("raw", Context.MODE_PRIVATE);
            Scanner scanner = new Scanner(new File(res, "hardware.xml"));
            String data = "";
            while (scanner.hasNextLine()) {
                data += scanner.nextLine() + "\n";
            }
            robotConfigFileManager.writeToFile(file, false, data);
            robotConfigFileManager.setActiveConfig(false, file);
            return true;
        } catch (IOException | RobotCoreException ex) {
            RobotLog.ww("Hardware Map Writer", ex, "Problem occurred while writing hardware map file to phone.");
            return false;
        }
    }

}
