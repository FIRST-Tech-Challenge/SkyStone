package org.firstinspires.ftc.teamcode.PID;

import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.PID.DriveConstantsPID;


public class RobotLogger {
    public static void dd(String tag, String format, Object... args)
    {
        if (DriveConstantsPID.ENABLE_LOGGING)
            RobotLog.dd(tag, String.format(format, args));
    }
    public static void dd(String tag, String message)
    {
        if (DriveConstantsPID.ENABLE_LOGGING)
            RobotLog.dd(tag, message);
    }

    public static void e(String format, Object... args)
    {
        if (DriveConstantsPID.ENABLE_LOGGING)
            RobotLog.e(String.format(format, args));
    }

}
