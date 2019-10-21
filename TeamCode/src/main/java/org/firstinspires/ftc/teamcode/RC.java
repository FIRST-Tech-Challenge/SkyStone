package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.internal.GlobalValuesActivity;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.roboticslibrary.FXTTelemetry;

/**
 * Created by FIXIT on 15-08-21.
 */
public class RC {

    public static OpMode o;
    public static LinearOpMode l;
    public static FXTTelemetry t;
    public static HardwareMap h;
    public static int runNum = 0;
    public final static String VUFORIA_LICENSE_KEY = "Ad0I0ir/////AAAAAfR3NIO1HkxSqM8NPhlEftFXtFAm6DC5w4Cjcy30" +
                                                    "WUdGozklFlAkxeHpjfWc4moeL2ZTPvZ+wAoyOnlZxyB6Wr1BRE9154j6K" +
                                                    "1/8tPvu21y5ke1MIbyoJ/5BAQuiwoAadjptZ8fpS7A0QGPrMe0VauJIM1" +
                                                    "mW3UU2ezYFSOcPghCOCvQ8zid1Bb8A92IkbLcBUcv3DEC6ia4SEkbRMY7" +
                                                    "TpOh2gzsXdsue4tqj9g7vj7zBU5Hu4WhkMDJRsThn+5QoHXqvavDsCElw" +
                                                    "mDHG3hlEYo7qN/vV9VcQUX9XnVLuDeZhkp885BHK5vAe8T9W3Vxj2H/R4" +
                                                    "oijQso6hEBaXsOpCHIWGcuphpoe9yoQlmNRRZ97";


    public static void setOpMode(OpMode op) {
        o = op;
        h = op.hardwareMap;
        t = new FXTTelemetry();
        t.setTelemetry(op.telemetry);

        if (op instanceof LinearOpMode) {
            l = (LinearOpMode) op;
        }//if

    }//setOpMode

    public static boolean globalBool(String key) {
        if (GlobalValuesActivity.globals.containsKey(key)) {
            return ((Boolean) GlobalValuesActivity.globals.get(key));
        }//if

        return false;
    }//globalBool

    public static String globalString(String key) {
        if (GlobalValuesActivity.globals.containsKey(key)) {
            return (String) GlobalValuesActivity.globals.get(key);
        }//if

        return "";
    }//globalString

    public static double globalDouble(String key) {
        if (GlobalValuesActivity.globals.containsKey(key)) {
            return ((Double) GlobalValuesActivity.globals.get(key));
        }//if

        return -1;
    }//globalDouble

    public static Object global(String key){
        return GlobalValuesActivity.globals.get(key);
    }

    public static String [] autoDashKeys(){
        String [] keys = new String[ GlobalValuesActivity.autoKeys.size()];
        GlobalValuesActivity.autoKeys.toArray(keys);
        return keys;
    }

    public static void setGlobalBool(String key, boolean val) {
        GlobalValuesActivity.add(key, val);
    }//globalBool

    public static void setGlobalString(String key, String val) {
        GlobalValuesActivity.add(key, val);
    }//globalString

    public static void setGlobalDouble(String key, double val) {
        GlobalValuesActivity.add(key, val);
    }//globalDouble


    public static Context c() {
        return AppUtil.getInstance().getActivity();
    }//context

    public static FtcRobotControllerActivity a() {
        return ((FtcRobotControllerActivity) AppUtil.getInstance().getActivity());
    }//activity

    public static void stop() {
        t.close();
    }//stop



}
