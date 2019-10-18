package org.firstinspires.ftc.teamcode.drive;

public final class MathUtil {
    public static double convert180to360(double angle) {
        if(angle > 0) return angle;
        else return 360 + angle;
    }
 }
