package org.firstinspires.ftc.teamcode.DutchFTCCore.Math;

public class Mathfunc {

    public static double clamp(double min, double max, double var){
        if (var < min){
            var = min;
        } else if (var > max) {
            var = max;
        }

        return var;
    }

    public static double[] clampProportion(double min, double max, double var0, double var1){
        double[] vars = new double[2];
        if(var0 > var1){
            if (var0 > max){
                var0 = max;
                var1 = (var1 * max)/var0;
            }
            if (var1 < min){
                var1 = max;
                var0 = (var0 * max)/var1;
            }
        }else if(var0 < var1){
            if (var1 > max){
                var0 = max;
                var1 = (var1 * max)/var0;
            }
            if (var0 < min){
                var1 = max;
                var0 = (var0 * max)/var1;
            }
        }

        vars[0] = var0;
        vars[1] = var1;
        return vars;
    }

    public static boolean range(double offset, double var, double goal){
        return var > goal - offset && var < goal + offset;
    }

    public static double FixAngle (double angle){
        while(angle < -180){
            angle += 360;
        }
        while (angle > 180) {
            angle -= 360;
        }
        return angle;
    }




}
