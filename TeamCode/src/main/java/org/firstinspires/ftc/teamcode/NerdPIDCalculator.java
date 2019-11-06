package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * This file contains the class for generic PID calculator we will be using to calculate PIDs.
 * As of now, it supports 2 kinds of input devices. Gyro and Motor Encoders.
 */

public class NerdPIDCalculator{
//Declare some variables that will be used later
    private final String instanceName;
    private double kP;
    private double kI;
    private double kD;


    private double previousTime = 0.0;
    private double currentError = 0.0;
    private double totalError = 0.0;
    private double pidTarget = 0.0;
    private double input = 0.0;
    private double output = 0.0;

    private double pOutput;
    private double iOutput;
    private double dOutput;

    private double prevDeviceInput = 0.0;


    private boolean debugFlag = false;

    private ElapsedTime elapsedTime = new ElapsedTime();

    //Constructor to create new NerdPIDCalculator object.

    public NerdPIDCalculator(
            final String instanceName,
            double       kP,
            double       kI,
            double       kD)

    {
        this.instanceName = instanceName;
        //Find the absolute value of proportional, integral, and derivative
        this.kP = Math.abs(kP);
        this.kI = Math.abs(kI);
        this.kD = Math.abs(kD);
        //Send info to console
        if (debugFlag)
            RobotLog.d("NerdPIDCalculator - Gains : %s -  %f | %f | %f ", this.instanceName, this.kP, this.kI, this.kD);
    }

    // Function to set the gains.

    public void setPIDGains(double kP, double kI, double kD)
    {
        final String funcName = "setPIDGains";

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        if (debugFlag)
            RobotLog.d("NerdPIDCalculator - Gains : %s -  %f | %f | %f ", this.instanceName, this.kP, this.kI, this.kD);
    }

    //Function to set the targets for PID

    public void setTarget(double target, double currentDeviceInput)
    {
        final String funcName = "setTarget";


            //
            // Target is absolute, use as is.
            //
            this.pidTarget = target;
            this.currentError = this.pidTarget - currentDeviceInput;


            totalError = 0.0;
            previousTime = elapsedTime.seconds();
            prevDeviceInput = currentDeviceInput;

            if (debugFlag)
                RobotLog.d("NerdPIDCalculator - %s, %s : Target %f, currentError %f ", this.instanceName, funcName,this.pidTarget, this.currentError);

    }

    //Function to reset the calculator, so that it can be used again.

    public void reset()
    {
        final String funcName = "reset";


        currentError = 0.0;
        previousTime = 0.0;
        totalError = 0.0;
        pidTarget = 0.0;
        output = 0.0;

    }

    //function to calculate error
    //Device type 1 = GYRO, anything else is like Encoders etc.

    public double getError(double deviceInput, int deviceType) {
        double robotError;

        robotError = pidTarget - deviceInput;
        if(deviceType == 1){
            // If measuring angle calculate error in -179 to +180 range  (

            while (robotError > 180)  robotError -= 360;
            while (robotError <= -180) robotError += 360;
        }
        return robotError;
    }

   // Function that calculates and returns PID output

    public double getOutput( double deviceInput, int deviceType)
    {

        final String funcName = "getOutput";

        //Before we get the error for this cycle, store the error from previous cycle in previous error.
        double prevError = currentError;
        //Store the current time. Will use this for calculating time between 2 cycles - delta time.
        double currTime = elapsedTime.seconds();
        //Find delta time, difference between time in this cycle and the previous.
        double deltaTime = currTime - previousTime;
        //Store the current time into previous time for using in next cycle.
         previousTime = currTime;
         //Call function to get the error based ont the set target and device input
         currentError = getError(deviceInput, deviceType);

         //Total error for finding I component is sum of errors in each cycle multiplied by delta time.
        totalError += (currentError * deltaTime);
        //Calculate P, I and D outputs.
        pOutput = kP*currentError;
        iOutput = kI*totalError;
        // dOutput = deltaTime > 0.0? kD*(currentError - prevError)/(deltaTime * 1000): 0.0;
        dOutput = deltaTime > 0.0? kD*(deviceInput - prevDeviceInput)/(deltaTime * 1000): 0.0;
        prevDeviceInput = deviceInput;

        //Total PID output
        output = pOutput + iOutput + dOutput;

        if (debugFlag)
            RobotLog.d("NerdPIDCalculator - %s - %s : prevError |  currentError | totalError | currTime | deltaTime | pOutput | iOutput |dOutput |output",
                this.instanceName, funcName);

        if (debugFlag)
            RobotLog.d("NerdPIDCalculator - %s - %s : %f | %f | %f | %f | %f | %f | %f | %f | %f ",
                this.instanceName, funcName,prevError, currentError, totalError, currTime, deltaTime, pOutput, iOutput,dOutput,output);

        //Return the output to the caller.
        return output;
    }

    public void setDebug(boolean debugFlag){
        this.debugFlag=debugFlag;
    }

}
