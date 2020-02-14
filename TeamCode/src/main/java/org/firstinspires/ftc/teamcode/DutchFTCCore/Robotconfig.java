package org.firstinspires.ftc.teamcode.DutchFTCCore;

public class Robotconfig {

    //drivetrain motors
    public static String MotorBackLeft = "MotorBackLeft";
    public static String MotorFrontLeft = "MotorFrontLeft";
    public static String MotorFrontRight = "MotorFrontRight";
    public static String MotorBackRight = "MotorBackRight";
    public static String MotorMiddle = "MotorMiddle";

    //TeleOp variables;
    public static double ShortCutSpeed = 0.3;

    //servo ports
    public static String Servo0 = "Servo0";
    public static String Servo1 = "Servo1";
    public static String Servo2 = "Servo2";
    public static String Servo3 = "Servo3";
    public static String Servo4 = "Servo4";
    public static String Servo5 = "Servo5";

    //drivetrains
    public static Drivetraintypes.Drivetrains DriveTrain = Drivetraintypes.Drivetrains.MECHANUMDRIVE;

    //logging
    public static String teamName = "FTCunits";
    public static boolean isLoggingUsed;

    //guidance subsystem
    public static boolean posControl;
    public static boolean angleControl = true;
    public static double angleKp = 1/20;
    public static double angleKi = 1/100;
    public static double angleKd = 1/25;
}
