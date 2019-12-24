package org.firstinspires.ftc.teamcode.Skystone.MotionProfiler;
import java.util.Vector;

public class SplineGenerator {
    double[][] outputData;
    public SplineGenerator(double[][] data){
        Profiler v = new Profiler(data);
        v.vehiclePath();
        outputData = new double[v.xValues.size()+1][2];
        for(int i = 0;i<v.xValues.size();i++){
            outputData[i][0] = v.xValues.get(i);
            outputData[i][1] = v.yValues.get(i);
        }
        outputData[v.xValues.size()][0] = data[data.length-1][0];
        outputData[v.xValues.size()][1] = data[data.length-1][1];
    }
    public double[][] getOutputData(){
        return outputData;
    }
}
class Profiler {
    // field variables
    private double[][] data;
    private Vector<Double> u = new Vector<>();
    Vector<Double> xValues = new Vector<>();
    Vector<Double> yValues = new Vector<>();
    /**
     * Profiler constructor, sets data matrix
     * @param data waypoints
     */
    public Profiler(double[][] data){
        this.data = data;
    }
    /**
     * generates path
     * called vehiclePath because it will be implemented into
     * robot navigation
     */
    public void vehiclePath(){
        // u is an arraylist that starts at 0
        // increases by 0.01 each index
        // and ends at 0.
        // the increments are based off of uIncrement
        double uIncrement = 0.2;
        for(double i = 0;i<=(1-uIncrement);i+=uIncrement){
            u.add(i);
        }
        /**
         * iterates through each 2 adjacent waypoints
         */
        for(int i = 0;i<data.length-1;i++){
            // iterate through each value of u
            for(int j = 0;j<1/uIncrement-1;j++){
                // simply used to calculate easier
                double uSquared = Math.pow(u.get(j),2);
                double uCubed = Math.pow(u.get(j),3);
                // plug it into a parametric cubic hermite spline equation
                xValues.add((2 * uCubed - 3 * uSquared + 1) * data[i][0] + (-2 * uCubed + 3 * uSquared) * data[i+1][0] + (uCubed - 2 * uSquared + u.get(j)) * data[i][2] + (uCubed - uSquared) * data[i+1][2]);
                yValues.add((2 * uCubed - 3 * uSquared + 1) * data[i][1] + (-2 * uCubed + 3 * uSquared) * data[i+1][1] + (uCubed - 2 * uSquared + u.get(j)) * data[i][3] + (uCubed - uSquared) * data[i+1][3]);
            }
        }
        // we have not accounted for the last point
        // the last point is the last waypoint coordinate
        xValues.add(data[data.length-1][0]);
        yValues.add(data[data.length-1][1]);
    }
}
