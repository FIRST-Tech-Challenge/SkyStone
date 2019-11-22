package org.firstinspires.ftc.teamcode.util;

public class Comparison {

    private static double epsilon = 0.001;

    public static boolean equalToEpsilon(double a, double b){
        if(((a + epsilon) >= b && (a - epsilon) <= b) || ((b + epsilon) >= a && (b - epsilon) <= a)){
            return true;
        }
        return false;
    }

}
