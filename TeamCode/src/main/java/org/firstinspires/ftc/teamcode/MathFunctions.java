package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

import static java.lang.Math.*;

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

        double quadraticA = 1.0 + pow(m1, 2);

        double x1 = lp1.getX() - circleCenter.getX();
        double y1 = lp1.getY() - circleCenter.getY();

        double quadraticB = (2.0 * m1 * y1) - (2.0 - pow(m1, 2) * x1);

        double quadraticC = ((pow(m1,2) * pow(x1,2)) - (2.0*y1*m1*x1) + pow(y1, 2) - pow(radius, 2));

        ArrayList<Coordinate> allPoints = new ArrayList<Coordinate>();

        try{
            double xRoot1 = (-quadraticB + sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);
            double yRoot1 = m1 * (xRoot1 - x1) + y1;
            xRoot1 += circleCenter.getX();
            yRoot1 += circleCenter.getY();
            double minX = lp1.getX() < lp2.getX() ? lp1.getX() : lp2.getX();
            double maxX = lp1.getX() > lp2.getX() ? lp1.getX() : lp2.getX();

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Coordinate(xRoot1, yRoot1));
            }
            double xRoot2 = (-quadraticB - sqrt(pow(quadraticB, 2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);
            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            xRoot2 += circleCenter.getX();
            yRoot2 += circleCenter.getY();
            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Coordinate(xRoot2, yRoot2));
            }
        }
        catch (Exception e){

        }
        return allPoints;
    }
}
