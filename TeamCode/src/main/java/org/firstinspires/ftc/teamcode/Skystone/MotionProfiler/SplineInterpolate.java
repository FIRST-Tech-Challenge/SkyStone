package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;

import java.util.Vector;

public class SplineInterpolate{
    Vector<Double> X = new Vector<>();
    Vector<Double> Y = new Vector<>();
    public SplineInterpolate(double[][] point){

        final float c_numSamples = 10000;


        final double[][] points = point;

        System.out.println("2d interpolated values.  x = f(t), y = f(t)n");
        for (int i = 0; i < c_numSamples; ++i)
        {
            double percent = ((double)i) / ((c_numSamples - 1));
            double x = 0;
            double y = 0;

            double tx = (points.length -1) * percent;
            int index = (int)tx;
            double t = tx - Math.floor(tx);

            double[] A = GetIndexClamped(points,index-1).clone();
            double[] B = GetIndexClamped(points,index).clone();
            double[] C = GetIndexClamped(points,index+1).clone();
            double[] D = GetIndexClamped(points,index+2).clone();

            x = CubicHermite(A[0], B[0], C[0], D[0], t);
            y = CubicHermite(A[1], B[1], C[1], D[1], t);

            System.out.println(tx+ "= " + x + " " + y);
            X.add(x);
            Y.add(y);
        }
        System.out.println(X);
        System.out.println(Y);
    }

    public double[][] getPoints(){
        Vector<Double> newX = new Vector<>();
        Vector<Double> newY = new Vector<>();
        newX.add(0.0);
        newY.add(0.0);
        for(int i = 0;i<X.size();i++){
            newX.add(X.get(i));
            newY.add(Y.get(i));
        }
        newX.add(X.get(X.size()-1));
        newY.add(Y.get(Y.size()-1));
        double[][] out = new double[newX.size()][2];

        for(int i = 0; i<newX.size();i+=50){
            out[i][0] = newX.get(i);
            out[i][1] = newY.get(i);
        }
        return out;
    }

    public static double[] GetIndexClamped(double[][] points, int index)
    {
        if (index < 0) {
            return points[0];
        }else if (index >= points.length){
            return points[points.length-1];
        } else{
            return points[index];
        }
    }
    public static Double CubicHermite (double A, double B, double C, double D, double t)
    {
        double a = (-A/2.0 + (3.0*B)/2.0 - (3.0*C)/2.0 + D/2.0);
        double b = (A - (5.0*B)/2.0 + 2.0*C - D / 2.0);
        double c = (-A/2.0 + C/2.0);
        double d = B;

        return (a*t*t*t + b*t*t + c*t + d);
    }
}
