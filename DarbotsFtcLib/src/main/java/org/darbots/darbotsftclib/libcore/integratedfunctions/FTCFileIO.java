package org.darbots.darbotsftclib.libcore.integratedfunctions;


import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class FTCFileIO {
    public static String readFile(File file){
        String FileContent = ReadWriteFile.readFile(file);
        return FileContent;
    }
    public static void writeFile(File file, String content){
        ReadWriteFile.writeFile(file,content);
    }

    public static File getSettingFile(String fileName){
        return AppUtil.getInstance().getSettingsFile(fileName);
    }
    public static File getFirstFolderFile(String filename){
        return new File(AppUtil.FIRST_FOLDER,filename);
    }
    public static File getFirstFolder(){
        return AppUtil.FIRST_FOLDER;
    }
    public static File getLogFolder(){
        return AppUtil.LOG_FOLDER;
    }

    public static File getLogFolderFile(String fileName){
        return new File(AppUtil.LOG_FOLDER,fileName);
    }

}
