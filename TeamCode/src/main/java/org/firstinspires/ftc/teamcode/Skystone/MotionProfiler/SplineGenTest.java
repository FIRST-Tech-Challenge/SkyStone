package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;

import android.os.SystemClock;

import java.util.Arrays;

public class SplineGenTest {
    public static void main(String[] args){
        double[][] points = {
                {31,-2,0,0},
                {23,17,0,10},
                {24,20,0,10},
                {24,30,0,10},
                {24,67,0,10},
                {27,75,0,10}};
        SplineGenerator s = new SplineGenerator(points);
        System.out.println(Arrays.deepToString(s.getOutputData()));
    }
}
