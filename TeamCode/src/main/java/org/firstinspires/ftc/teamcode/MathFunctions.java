package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class MathFunctions {
    public static double AngleWrap(double angle){
        while(angle < -Math.PI){
            angle += 2 * Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2 * Math.PI;
        }
        return angle;
    }
    public static ArrayList<Coordinate> lineCircleIntersection(Coordinate circleCenter, double radius,
                                                               Coordinate lp1, Coordinate lp2){
        if(Math.abs(lp1.getY() - lp2.getY()) < 0.003){
            lp1.setY(lp2.getY() + 0.003);
        }
        if(Math.abs(lp1.getX() - lp2.getX()) < 0.003){
            lp1.setX(lp2.getX() + 0.003);
        }
        double m1 = (lp2.getY() - lp1.getY())/(lp2.getX() - lp1.getX());//check
        double quadraticA = 1.0 + Math.pow(m1, 2);
        double x1 = lp1.getX() - circleCenter.getX();
        double y1 = lp1.getY() - circleCenter.getY();
        double quadraticB = (2.0 * m1 * y1) - (2.0 - Math.pow(m1, 2) * x1);
        double quadraticC = ((Math.pow(m1,2) * Math.pow))
    }
}
