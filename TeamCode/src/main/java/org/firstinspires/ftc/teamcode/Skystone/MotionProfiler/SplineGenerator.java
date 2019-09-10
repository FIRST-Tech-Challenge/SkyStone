package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.getArrayOfRow;
import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.normalize;
import static org.firstinspires.ftc.teamcode.Skystone.MathFunctions.subtractVectors;

/**
 * Created by sam on 7/20/19.
 */
public class SplineGenerator {
    double[][] outputData;

    public SplineGenerator(double[][] data){
        Profiler v = new Profiler(data);
        v.vehiclePath();
        v.vehicleTraj();
        this.outputData = v.generateOutput();
    }

    public double[][] getOutputData(){
        return outputData;
    }
}

class Profiler {
    double[][] data;
    Vector<Double> uV = new Vector<>();
    Vector<Double> xV = new Vector<>();
    Vector<Double> yV = new Vector<>();
    Vector<Double> dxdu = new Vector<>();
    Vector<Double> dydu = new Vector<>();
    Vector<Double> u = new Vector<>();
    Vector<Double> xV1 = new Vector<>();
    Vector<Double> yV1 = new Vector<>();
    Vector<Double> dxdu1 = new Vector<>();
    Vector<Double> dydu1 = new Vector<>();
    Vector<Double> t = new Vector<>();
    double[] vCenter;
    double[] dCenter;
    double[][] leftWheel;
    double[][] rightWheel;
    double[] distance;
    double[] leftSpeed;
    double[] rightSpeed;
    double[] tV;
    double[] leftDistance;
    double[] rightDistance;
    double[] heading;

    double vMotorMax = 16.875164431332;
    double vMaxCenter = 16.87; // inch/second
    double tRamp = 0.5; // seconds
    double totalDist;
    double totalTime;

    public Profiler(double[][] data){
        this.data = data;
    }

    public void vehiclePath(){
        int numData = data.length;
        int Nu = 100;
        double distWheels = 14;

        for(double i = 0; i<=numData-1;i+=0.01){
            uV.add(i);
            xV.add(0.0);
            yV.add(0.0);
            dxdu.add(0.0);
            dydu.add(0.0);
        }
        xV.add(0.0);
        yV.add(0.0);
        dxdu.add(0.0);
        dydu.add(0.0);

        for(double i = 0;i<=(1-1/Nu);i+=0.01){
            u.add(i);
        }
        uV.clear();
        xV.clear();
        yV.clear();
        dxdu.clear();
        dydu.clear();
        for(int i = 0;i<numData-1;i++){
            for(int j = 0;j<u.size();j++){
                double u2 = Math.pow(u.get(j),2);
                double u3 = Math.pow(u.get(j),3);

                xV1.add((2 * u3 - 3 * u2 + 1) * data[i][0] + (-2 * u3 + 3 * u2) * data[i+1][0] + (u3 - 2 * u2 + u.get(j)) * data[i][2] + (u3 - u2) * data[i+1][2]);
                yV1.add((2 * u3 - 3 * u2 + 1) * data[i][1] + (-2 * u3 + 3 * u2) * data[i+1][1] + (u3 - 2 * u2 + u.get(j)) * data[i][3] + (u3 - u2) * data[i+1][3]);

                dxdu1.add((6 * u2 - 6 * u.get(j)) * (data[i][0] - data[i+1][0]) + data[i][2] * (3 * u2 - 4 * u.get(j)+ 1) + data[i+1][2] * (3 * u2 - 2 * u.get(j)));
                dydu1.add((6 * u2 - 6 * u.get(j)) * (data[i][1] - data[i+1][1]) + data[i][3] * (3 * u2 - 4 * u.get(j)+ 1) + data[i+1][3] * (3 * u2 - 2 * u.get(j)));
            }
            Vector<Double> stitchToUV = new Vector<>();

            for(int r = 0; r<u.size();r++){
                stitchToUV.add(u.get(r)+(i));
            }

            uV.addAll(stitchToUV);
            xV.addAll(xV1);
            yV.addAll(yV1);
            dxdu.addAll(dxdu1);
            dydu.addAll(dydu1);
        }
        uV.add(1.0);

        xV.add(data[data.length-1][0]);
        yV.add(data[data.length-1][1]);
        dxdu.add(data[data.length-1][2]);
        dydu.add(data[data.length-1][3]);

        distance = new double[uV.size()];

        for(int i = 0;i<xV.size()-1;i++){
            distance[i+1] = distance[i] +Math.sqrt(Math.pow(xV.get(i+1)- xV.get(i),2) + Math.pow((yV.get(i+1) - yV.get(i)),2));
        }

        leftWheel = new double[uV.size()][2];
        rightWheel = new double[uV.size()][2];

        for(int i = 0;i<uV.size();i++){
            double[] tanV = {dxdu.get(i), dydu.get(i)};

            for(int j = 0;j<2;j++){
                tanV[j] = tanV[j] / Math.sqrt(Math.pow(tanV[0],2) + Math.pow(tanV[1],2));
            }
            double[] leftV = {-1*tanV[1], tanV[0]};
            leftWheel[i][0] = xV.get(i) + leftV[0] * distWheels/2;
            rightWheel[i][0] = xV.get(i) - leftV[0] * distWheels/2;
            leftWheel[i][1] = yV.get(i) + leftV[1] * distWheels/2;
            rightWheel[i][1] = yV.get(i) - leftV[1] * distWheels/2;
        }
    }

    public void vehicleTraj() {
        System.out.println(distance[distance.length-1]);
        totalDist = distance[distance.length - 1];
        System.out.println(totalDist);
        totalTime = totalDist / vMaxCenter + tRamp;
        System.out.println(totalTime);
        for (double i = 0; i <= 1; i += 0.001) {
            t.add(i * totalTime);
        }
        vCenter = new double[t.size()];
        dCenter = new double[t.size()];

        for (int i = 0; i < t.size(); i++) {
            if (t.get(i) < tRamp) {
                vCenter[i] = (vMaxCenter / tRamp) * t.get(i);
            } else if (t.get(i) < totalTime - tRamp) {
                vCenter[i] = vMaxCenter;
            } else if (t.get(i) < totalTime) {
                vCenter[i] = (-1 * vMaxCenter / tRamp) * (t.get(i) - totalTime);
            } else {
                vCenter[i] = 0;
            }
        }


        for (int i = 1; i < t.size(); i++) {
            dCenter[i] = dCenter[i - 1] + (vCenter[i - 1] + vCenter[i]) / 2 * (t.get(i) - t.get(i - 1));
        }
        tV = new double[uV.size()];
        leftSpeed = new double[uV.size()];
        rightSpeed = new double[uV.size()];
        for (int i = 0; i < tV.length - 1; i++) {
            tV[i] = (dCenter[i + 1] - dCenter[i]) / (t.get(i + 1) - t.get(i)) * (distance[i] - t.get(i)) + dCenter[i];
        }

        tV[tV.length - 1] = totalTime;
        double s1;
        double s2;
        for (int i = 1; i < tV.length - 1; i++) {
            Vector<Double> leftI = new Vector<>(getArrayOfRow(leftWheel, i));
            Vector<Double> leftIMinus1 = new Vector<>(getArrayOfRow(leftWheel, i - 1));
            Vector<Double> leftIPlus1 = new Vector<>(getArrayOfRow(leftWheel, i + 1));

            Vector<Double> rightI = new Vector<>(getArrayOfRow(rightWheel, i));
            Vector<Double> rightIMinus1 = new Vector<>(getArrayOfRow(rightWheel, i - 1));
            Vector<Double> rightIPlus1 = new Vector<>(getArrayOfRow(rightWheel, i + 1));

            s1 = normalize(subtractVectors(leftI, leftIMinus1)) / (tV[i] - tV[i - 1]);
            s2 = normalize(subtractVectors(leftIPlus1, leftI)) / (tV[i + 1] - tV[i]);

            leftSpeed[i] = (s1 + s2) / 2;

            s1 = normalize(subtractVectors(rightI, rightIMinus1)) / (tV[i] - tV[i - 1]);
            s2 = normalize(subtractVectors(rightIPlus1, rightI)) / (tV[i + 1] - tV[i]);

            rightSpeed[i] = (s1 + s2) / 2;
        }
        leftSpeed[0] = 0;
        leftSpeed[leftSpeed.length - 1] = 0;
        rightSpeed[0] = 0;
        rightSpeed[rightSpeed.length - 1] = 0;


        leftDistance = new double[uV.size()];
        rightDistance = new double[uV.size()];
        heading = new double[uV.size()];

        for (int i = 1; i < uV.size(); i++) {
            leftDistance[i] = leftDistance[i - 1] + ((leftSpeed[i] + leftSpeed[i - 1]) / 2 * (tV[i] - tV[i - 1]));
            rightDistance[i] = rightDistance[i - 1] + ((rightSpeed[i] + rightSpeed[i - 1]) / 2 * (tV[i] - tV[i - 1]));

        }

        for (int i = 0; i < uV.size(); i++) {
            if (dxdu.get(i) > 0) {
                heading[i] = 90 - Math.toDegrees(Math.atan(dydu.get(i) / dxdu.get(i)));
            } else if (dxdu.get(i) < 0) {
                heading[i] = -1 * Math.atan(dydu.get(i) / dxdu.get(i)) - 90;
            } else {
                heading[i] = 0;
            }
        }
        System.out.println(t);
    }



    public double[][] generateOutput(){
        double[][] out = new double[leftSpeed.length][7];

        for(int i = 0;i<leftSpeed.length;i++){
            out[i][0] = leftSpeed[i];
        }
        for(int i = 0;i<leftSpeed.length;i++){
            out[i][1] = rightSpeed[i];
        }
        for(int i = 0;i<leftSpeed.length;i++){
            out[i][2] = tV[i];
        }
        for(int i = 0;i<leftSpeed.length;i++){
            out[i][3] = distance[i];
        }
        for(int i = 0;i<leftSpeed.length;i++){
            out[i][4] = leftDistance[i];
        }
        for(int i = 0;i<leftSpeed.length;i++){
            out[i][5] = rightDistance[i];
        }
        for(int i = 0;i<leftSpeed.length;i++){
            out[i][6] = heading[i];
        }
        return out;
    }
}
