package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.Range;

public class NerdPIDCalculator{
//Declare some variables that will be used later
    private final String instanceName;
    private double kP;
    private double kI;
    private double kD;

    private double minTarget = 0.0;
    private double maxTarget = 0.0;


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

    public void setPIDGains(double kP, double kI, double kD)
    {
        final String funcName = "setPIDGains";

        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        if (debugFlag)
            RobotLog.d("NerdPIDCalculator - Gains : %s -  %f | %f | %f ", this.instanceName, this.kP, this.kI, this.kD);
    }

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
    //Device type 1 = GYRO, anything else is like Encoders etc
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
        double prevError = currentError;

        double currTime = elapsedTime.seconds();
        double deltaTime = currTime - previousTime;
         previousTime = currTime;
         currentError = getError(deviceInput, deviceType);

        totalError += (currentError * deltaTime);
        pOutput = kP*currentError;
        iOutput = kI*totalError;
       // dOutput = deltaTime > 0.0? kD*(currentError - prevError)/(deltaTime * 1000): 0.0;
        dOutput = deltaTime > 0.0? kD*(deviceInput - prevDeviceInput)/(deltaTime * 1000): 0.0;
        prevDeviceInput = deviceInput;

        output = pOutput + iOutput + dOutput;

        if (debugFlag)
            RobotLog.d("NerdPIDCalculator - %s - %s : prevError |  currentError | totalError | currTime | deltaTime | pOutput | iOutput |dOutput |output",
                this.instanceName, funcName);

        if (debugFlag)
            RobotLog.d("NerdPIDCalculator - %s - %s : %f | %f | %f | %f | %f | %f | %f | %f | %f ",
                this.instanceName, funcName,prevError, currentError, totalError, currTime, deltaTime, pOutput, iOutput,dOutput,output);


        return output;
    }

    public void setDebug(boolean debugFlag){
        this.debugFlag=debugFlag;
    }

}
