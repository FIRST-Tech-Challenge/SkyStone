package org.firstinspires.ftc.teamcode.util;

/**
 * Created by Windows on 2016-11-04.
 */
public class MathUtils {

    public static double max(double... values) {
        double max = 0;

        for (int i = 0; i < values.length; i++) {
            if (max < values[i]) {
                max = values[i];
            }//if
        }//for

        return max;
    }//max

    public static double min(double... values) {
        double min = 0;

        for (int i = 0; i < values.length; i++) {
            if (min > values[i]) {
                min = values[i];
            }//if
        }//for

        return min;
    }//min

    public static double range(double... values) {
        double max = 0;

        for (int i = 0; i < values.length; i++) {
            for (int j = i + 1; j < values.length; j++) {
                if (max < Math.abs(values[i] - values[j])) {
                    max = Math.abs(values[i] - values[j]);
                }//if
            }//for
        }//for

        return max;
    }//max

    public static int getQuadrant(double x, double y){
        if(x >= 0){
            if(y >= 0) return 1;
            else return 4;
        } else {
            if(y >= 0) return 2;
            else return 3;
        }
    }


    public static double roundToNearest(double num, double increment, double lowBound, double upBound){
        for (int i = 0; lowBound + i * increment <= upBound; ) {
            if(num < lowBound + (i + 0.5) * increment){
                return lowBound + i * increment;
            }
            i++;
        }
        throw new IllegalArgumentException("num " + num + " not in range! Range is " + lowBound +"-" + upBound);
    }

    public static double roundToNearest(double num, double increment, double lowBound){

        num -= lowBound;
        num = increment * Math.round(num / increment);

        return num + lowBound;
    }

    public static boolean inRange(double num, double bound1, double bound2){
        return (num > bound1 && num < bound2) || (num > bound2 && num < bound1);
    }

    //currently for angles from IMU only
    public static double getReflexiveAngle(double angdeg) {
        angdeg += 90;

        if (angdeg > 360) {
            angdeg -= 360;
        }//if

        if (angdeg > 270) {
            return 360 - angdeg;
        } else if (angdeg > 180) {
            return angdeg - 180;
        } else if (angdeg > 90) {
            return 180 - angdeg;
        } else {
            return angdeg;
        }//else
    }

    public static double cvtAngleToNewDomain(double angle) {
        if (angle < -180) {
            angle += 360;
        } else if (angle > 180) {
            angle -= 360;
        }//elseif

        return angle;
    }

    public static double cvtAngleJumpToNewDomain(double delta) {
        if (Math.abs(delta) > 180) {
            delta = -(Math.signum(delta) * 360 - delta);
        }//if

        return delta;
    }
}
