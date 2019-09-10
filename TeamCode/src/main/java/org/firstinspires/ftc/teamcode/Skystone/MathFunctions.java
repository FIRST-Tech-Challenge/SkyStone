package org.firstinspires.ftc.teamcode.Skystone;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import java.util.Vector;
import static java.lang.Math.*;

public class MathFunctions {

    public static double angleWrap (double angle) {return angle % (2 * Math.PI );
    }

    public static Vector<Double> getArrayOfRow(double[][] matrix, int row){
        Vector<Double> out = new Vector<>();
        for(int i = 0;i<matrix[row].length;i++){
            out.add(matrix[row][i]);
        }
        return out;
    }

    public static Vector<Double> addToVector(Vector<Double> one, double two){
        for(int i = 0;i<one.size();i++){
            one.set(i,one.get(i)+two);
        }
        return one;
    }

    public static Vector<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2){
        //make sure the points don't exactly line up so the slopes work
        if(Math.abs(linePoint1.y- linePoint2.y) < 0.003){
            linePoint1.y = linePoint2.y + 0.003;
        }
        if(Math.abs(linePoint1.x- linePoint2.x) < 0.003){
            linePoint1.x = linePoint2.x + 0.003;
        }

        //calculate the slope of the line
        double m1 = (linePoint2.y - linePoint1.y)/(linePoint2.x-linePoint1.x);

        double quadraticA = 1.0 + pow(m1,2);

        //shift one of the line's points so it is relative to the circle
        double x1 = linePoint1.x-circleCenter.x;
        double y1 = linePoint1.y-circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * pow(m1,2) * x1);

        double quadraticC = ((pow(m1,2)*pow(x1,2)) - (2.0*y1*m1*x1) + pow(y1,2)-pow(radius,2));


        Vector<Point> allPoints = new Vector<>();

        try{
            // solve roots
            double xRoot1 = (-quadraticB + Math.sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            double yRoot1 = m1 * (xRoot1 - x1) + y1;


            //add back in translations
            xRoot1 += circleCenter.x;
            yRoot1 += circleCenter.y;

            //within range
            double minX = linePoint1.x < linePoint2.x ? linePoint1.x : linePoint2.x;
            double maxX = linePoint1.x > linePoint2.x ? linePoint1.x : linePoint2.x;

            if(xRoot1 > minX && xRoot1 < maxX){
                allPoints.add(new Point(xRoot1,yRoot1));
            }

            //do the same for the other root
            double xRoot2 = (-quadraticB - Math.sqrt(pow(quadraticB,2) - (4.0 * quadraticA * quadraticC)))/(2.0*quadraticA);

            double yRoot2 = m1 * (xRoot2 - x1) + y1;
            //now we can add back in translations
            xRoot2 += circleCenter.x;
            yRoot2 += circleCenter.y;

            //make sure it was within range of the segment
            if(xRoot2 > minX && xRoot2 < maxX){
                allPoints.add(new Point(xRoot2,yRoot2));
            }

        }catch(Exception e){
            //if there are no roots
        }
        return allPoints;
    }

    public static Vector<Double> subFromVector(Vector<Double> one, double two){
        for(int i = 0;i<one.size();i++){
            one.set(i,one.get(i)-two);
        }
        return one;
    }

    public static Vector<Double> multToVector(Vector<Double> one, double two){
        for(int i = 0;i<one.size();i++){
            one.set(i,one.get(i)*two);
        }
        return one;
    }

    public static Vector<Double> divideFromVector(Vector<Double> one, double two){
        for(int i = 0;i<one.size();i++){
            one.set(i,one.get(i)/two);
        }
        return one;
    }

    public static Vector<Double> subtractVectors(Vector<Double> one, Vector<Double> two){
        for(int i = 0;i<one.size();i++){
            one.set(i,one.get(i)-two.get(i));
        }
        return one;
    }

    public static Vector<Double> multiplyVectors(Vector<Double> one, Vector<Double> two){
        for(int i = 0;i<one.size();i++){
            one.set(i,one.get(i)*two.get(i));
        }
        return one;
    }

    public static Vector<Double> divideVectors(Vector<Double> one, Vector<Double> two){
        for(int i = 0;i<one.size();i++){
            one.set(i,one.get(i)*two.get(i));
        }
        return one;
    }

    public static Vector<Double> addVectors(Vector<Double> one, Vector<Double> two){
        for(int i = 0;i<one.size();i++){
            one.set(i,one.get(i)+two.get(i));
        }
        return one;
    }

    public static double normalize(Vector<Double> v){
        double sum = 0;
        for(int i = 0;i<v.size();i++){
            sum+= pow(v.get(i),2);
        }
        return java.lang.Math.sqrt(sum);
    }

    public static double[][] matrixMultiplication(double[][] a, double[][] b) {
        int rowsInA = a.length;
        int columnsInA = a[0].length; // same as rows in B
        int columnsInB = b[0].length;
        double[][] c = new double[rowsInA][columnsInB];
        for (int i = 0; i < rowsInA; i++) {
            for (int j = 0; j < columnsInB; j++) {
                for (int k = 0; k < columnsInA; k++) {
                    c[i][j] = c[i][j] + a[i][k] * b[k][j];
                }
            }
        }
        return c;
    }
}
