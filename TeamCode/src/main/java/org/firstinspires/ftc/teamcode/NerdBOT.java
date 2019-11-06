/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * This file contains definition of class for creating NerdBOT class.
 * This is represenging our robot for this years FTC competition.
 * This class has functions to drive using inverse kinematics using X, Y and Z inputs.
 * X, and Y will be the distance in X and Y direction in inches and Z will be angle to hold.
 **/

public class NerdBOT{

    private boolean debugFlag=false;

    // Robot has 4 motors

   private DcMotor leftMotor;
   private DcMotor rightMotor;
   private DcMotor leftMotorB;
   private DcMotor rightMotorB;

   private ElapsedTime runtime = new ElapsedTime();
   private BNO055IMU imu = null;   // Gyro device

    //We need an opmode to get the hardware map etc.

    LinearOpMode opmode;


    //Initial Speed for Robot to run
    private double minSpeed = 0.1;
    private double maxSpeed = 1.0;

    private final double ticksPerRotation = 560.0; //For omni wheels we are using
    private final double wheelDiameter = 3.54331; // For omni wheels we are using
    private final double GEAR_RATIO = 20.0/15.0;  // Gear ratio


    private NerdPIDCalculator zPIDCalculator ; // Variable to hold PID calculator for Z while driving
    private NerdPIDCalculator turnPIDCalculator ; //Variable to hold PID calculator for Turning to an angle
    private  NerdPIDCalculator xPIDCalculator ; //Variable to hold PID calculator for X while driving
    private NerdPIDCalculator yPIDCalculator ; //Variable to hold PID calculator for Y while driving

    private HardwareMap hardwareMap;

    static final double HEADING_THRESHOLD = 2;
    static final double DISTANCE_THRESHOLD = 25;


    static  final int GYRO = 1;
    static final int ENCODERS = 2;

    /**Constructor to create NerdBOT object
     *
     * Creates a new NerdBOT object and assigns the hardwareMap provided by caller
     * @param   opmode  Hardware Map provided by the calling OpMode.
     *                  NerdBOT takes an opmode object so that it can get the hardwareMap.     *
     */

    public  NerdBOT(LinearOpMode opmode){
        this.opmode = opmode;
        this.hardwareMap = opmode.hardwareMap;
    }


    /** Function to drive robot based on PIDs in X, Y and Z directions.
     *     *
     * @param   speed               - Initial Speed. Not used at this time.
     * @param   xDistance           - Distance to move in X direction.
     * @param   yDistance           - Distance to move in Y direction.
     * @param   zAngleToMaintain    - Angle to maintain.
     */

    public void nerdPidDrive(double speed, double xDistance,double yDistance, double zAngleToMaintain) {

        final String funcName = "nerdPidDrive";

        //Speeds for assigning to 4 motors.

        double leftSpeed;
        double rightSpeed;
        double rightSpeedB;
        double leftSpeedB;

        //To hold the motor encoder ticks in each direction.

        int xTicks,yTicks;

        //Hold the PID values for X,Y and Z.
        double zpid, xpid, ypid;

        //Holds final motor powers to be sent to motors.

        double [] motorPowers;

        //Reset the timer
        runtime.reset();

        //Reset the Calculators

        xPIDCalculator.reset();
        yPIDCalculator.reset();
        zPIDCalculator.reset();

        //Reset the motors so that the encoders are set to 0.

        motorsResetAndRunUsingEncoders();

        //Convert X and Y distances to corresponding encoder ticks.

        xTicks = xDistance != 0.0 ? (int)inchesToTicksForQuadStraightDrive(wheelDiameter,xDistance, 45.0): 0;
        yTicks = yDistance != 0.0 ? (int)inchesToTicksForQuadStraightDrive(wheelDiameter, yDistance, 45.0): 0;

        if (debugFlag)
            RobotLog.d ("NerdBOT - xTicks = %d, yTicks = %d , Angle %f", xTicks, yTicks, zAngleToMaintain);

        //Set PID targets for X, Y and Z

        xPIDCalculator.setTarget(xTicks,findXDisplacement());
        yPIDCalculator.setTarget(yTicks,findYDisplacement());
        zPIDCalculator.setTarget(zAngleToMaintain,getZAngleValue());


        if (debugFlag)
            RobotLog.d ("NerdBOT - opModeIsActive NOTCHECKED = %b, distanceTargetReached = %b",  this.opmode.opModeIsActive(), distanceTargetReached(xTicks,yTicks));

        //Perform PID Loop until we reach the targets
//
        while ( this.opmode.opModeIsActive() && (( !distanceTargetReached(xTicks,yTicks) ))){

           if(debugFlag)
               RobotLog.d ("NerdBOT - opModeIsActive  Inside While Loop = %b, distanceTargetReached = %b",  this.opmode.opModeIsActive(), distanceTargetReached(xTicks,yTicks));

            //Feed the input device readings to corresponding PID calculators:

            zpid = zPIDCalculator.getOutput(getZAngleValue(),GYRO);
            xpid = xPIDCalculator.getOutput(findXDisplacement(),ENCODERS);
            ypid = yPIDCalculator.getOutput(findYDisplacement(),ENCODERS);

            if (debugFlag)
                RobotLog.d("NerdBOT XPID - %f, YPID - %f , ZPID - %f", xpid, ypid, zpid);

            //Calculate Speeds based on Inverse Kinematics

            leftSpeed = ypid - zpid + xpid;
            rightSpeed = ypid + zpid - xpid;
            leftSpeedB = ypid - zpid - xpid;
            rightSpeedB = ypid + zpid + xpid;

            if (debugFlag) {
                RobotLog.d("NerdBOT  - Speeds before Normalizing : %s |leftSpeed | rightSpeed | leftSpeedB | rightSpeedB ", funcName);
                RobotLog.d("NerdBOT  - Speeds before Normalizing : %s |%f|%f|%f|%f ", funcName, leftSpeed, rightSpeed, leftSpeedB, rightSpeedB);
            }

            //Normalize the Motor Speeds for Min and Max Values

           motorPowers = normalizeSpeedsForMinMaxValues(leftSpeed,rightSpeed,rightSpeedB,leftSpeedB);

           // Set Powers to corresponding Motors

           leftMotor.setPower(motorPowers[0]);
           rightMotor.setPower(motorPowers[1]);
           rightMotorB.setPower(motorPowers[2]);
           leftMotorB.setPower(motorPowers[3]);


            if (debugFlag) {
                RobotLog.d("NerdBOT  - Speeds after Normalizing : %s |leftSpeed | rightSpeed | leftSpeedB | rightSpeedB ", funcName);
                RobotLog.d("NerdBOT  - Speeds after Normalizing : %s |%f|%f|%f|%f ", funcName, motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);
            }

        }

        //Brake once the PID loop is complete

       brakeMotorsAndStop();

    }

    public void nerdPidTurn(double turnspeed, double targetAngle) {

        final String funcName = "nerdPidTurn";

        Orientation angles;
        double pidvalue;
        double [] motorPowers;


        double leftSpeed=0, rightSpeed=0, rightSpeedB=0, leftSpeedB=0;

        runtime.reset();

        turnPIDCalculator.setTarget(targetAngle, getZAngleValue());

        motorsSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (this.opmode.opModeIsActive() && (!onTarget(getZAngleValue()))) {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                pidvalue = turnPIDCalculator.getOutput(angles.firstAngle, 1);


                leftSpeedB =  -pidvalue;
                rightSpeedB =  pidvalue;
                leftSpeed =  -pidvalue;
                rightSpeed = pidvalue;

              if (debugFlag) {
                  RobotLog.d("NerdBOT - Speeds before Normalizing : %s |leftSpeed | rightSpeed | leftSpeedB | rightSpeedB ", funcName);
                  RobotLog.d("NerdBOT - Speeds before Normalizing : %s |%f|%f|%f|%f", funcName,leftSpeed,rightSpeed,leftSpeedB,rightSpeedB);
              }



                motorPowers = normalizeSpeedsForMinMaxValues(leftSpeed,rightSpeed,rightSpeedB,leftSpeedB);


                leftMotor.setPower(motorPowers[0]);
                rightMotor.setPower(motorPowers[1]);
                rightMotorB.setPower(motorPowers[2]);
                leftMotorB.setPower(motorPowers[3]);

                //////////////////////////////////////////////////////////////////////////////////

              if (debugFlag) {
                  RobotLog.d("NerdBOT -  %s : current Angle |  pidValue ", funcName);
                  RobotLog.d ("NerdBOT - %s  | %f |%f ", funcName,imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, pidvalue);
                  RobotLog.d("NerdBOT - Speeds after Normalizing : %s |leftSpeed | rightSpeed | leftSpeedB | rightSpeedB ", funcName);
                  RobotLog.d("NerdBOT - Speeds after Normalizing : %s |%f|%f|%f|%f ", funcName,motorPowers[0],motorPowers[1],motorPowers[2],motorPowers[3]);
              }
            }

            brakeMotorsAndStop();
    }


    public int inchesToTicksForQuadStraightDrive(double wheelDiameter, double straightDistance, double wheelMountAngle){
        int ticks;

        //circumference of the wheel

        double circum = wheelDiameter * 3.14;

        //Since the wheels are mounted at an angle, do the math for wheel distance to travel
        double wheelDistanceToTravel = (straightDistance * Math.cos(Math.toRadians(wheelMountAngle))) * GEAR_RATIO ;
        if (debugFlag)
            RobotLog.d("NerdBOT - inchesToTicksForQuadStraightDrive - wheelDistanceToTravel - %f, straightDistance - %f",wheelDistanceToTravel,straightDistance );

       //Find the number of rotations for wheel distance to travel.
        double numberOfWheelRotations = wheelDistanceToTravel/circum;

        if (debugFlag)
            RobotLog.d("NerdBOT -inchesToTicksForQuadStraightDrive - numberOfWheelRotations - %f",numberOfWheelRotations );

        //Convert number of rotation into motor encoder ticks
        ticks =  (int)(Math.round(ticksPerRotation * numberOfWheelRotations));
        if (debugFlag)
            RobotLog.d("NerdBOT - inchesToTicksForQuadStraightDrive - ticks - %d",ticks );

        return ticks;
    }

    //Function to find if the robot is within desired tolerance for Z angle.

    boolean onTarget(double angle) {
        double error;
        boolean onTarget = false;


        // determine turn power based on +/- error
        error = turnPIDCalculator.getError(angle,1);

        if (Math.abs(error) <= HEADING_THRESHOLD) {

            onTarget = true;
        }

        return onTarget;
    }

    //Function to find out the Robot travel distance in X direction.

    double findXDisplacement(){

            return (leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition()
                    -leftMotorB.getCurrentPosition() + rightMotorB.getCurrentPosition())/4.0;

    }

    //Function to find out the Robot travel distance in Y direction.


    double findYDisplacement(){

        return (this.leftMotor.getCurrentPosition() + this.rightMotor.getCurrentPosition()
                + this.leftMotorB.getCurrentPosition() + this.rightMotorB.getCurrentPosition())/4.0;

    }

    //Function to initialize hardware components.


    public void initializeHardware(){

            //Initialize Motors

        this.leftMotor = this.hardwareMap.dcMotor.get("Front_Left_Motor");
        this.rightMotor = this.hardwareMap.dcMotor.get("Front_Right_Motor");
        this.leftMotorB = this.hardwareMap.dcMotor.get("Rear_Left_Motor");
        this.rightMotorB = this.hardwareMap.dcMotor.get("Rear_Right_Motor");

        this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftMotorB.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        this.imu = this.hardwareMap.get(BNO055IMU.class, "imu");
        this.imu.initialize(parameters);
        this.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void motorsResetAndRunUsingEncoders(){

        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void motorsRunToPositionUsingEncoders(int ticksToMove, boolean resetEncoders){

            this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.leftMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.rightMotorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(resetEncoders) {
                this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.leftMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                this.rightMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                this.leftMotor.setTargetPosition(ticksToMove);
                this.rightMotor.setTargetPosition(ticksToMove);
                this.leftMotorB.setTargetPosition(ticksToMove);
                this.rightMotorB.setTargetPosition(ticksToMove);
            }else{

                this.leftMotor.setTargetPosition(this.leftMotor.getCurrentPosition()+ ticksToMove);
                this.rightMotor.setTargetPosition(this.rightMotor.getCurrentPosition()+ ticksToMove);
                this.leftMotorB.setTargetPosition(this.leftMotorB.getCurrentPosition()+ ticksToMove);
                this.rightMotorB.setTargetPosition(this.rightMotorB.getCurrentPosition() + ticksToMove);
            }


            this.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void brakeMotorsAndStop(){

        this.leftMotor.setPower(0.0);
        this.rightMotor.setPower(0.0);
        this.leftMotorB.setPower(0.0);
        this.rightMotorB.setPower(0.0);

    }

    public void motorsSetMode(DcMotor.RunMode runMode){

            this.leftMotor.setMode(runMode);
            this.rightMotor.setMode(runMode);
            this.rightMotorB.setMode(runMode);
            this.leftMotorB.setMode(runMode);
    }

    //Function to reduce the speed in case of the PID values are greater than max speed.
    //A minimum speed also can be assigned if minimum speed is more than 0.

    public double[] normalizeSpeedsForMinMaxValues(double leftSpeed, double rightSpeed, double rightSpeedB, double leftSpeedB){

        double max = Math.max(Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed)), Math.max(Math.abs(leftSpeedB), Math.abs(rightSpeedB)));
        if (max > maxSpeed) {
            leftSpeed = (leftSpeed * this.maxSpeed)/max;
            rightSpeed =(rightSpeed * this.maxSpeed)/ max;
            leftSpeedB = (leftSpeedB * this.maxSpeed)/max;
            rightSpeedB = (rightSpeedB * this.maxSpeed)/max;
        }

        //If there is min speed other than 0.
        // Preserve the sign of each speed for each motor
        int leftSign, rightSign, leftSignB, rightSignB;

        leftSign = (int)Math.signum(leftSpeed);
        rightSign = (int)Math.signum(rightSpeed);
        leftSignB =(int) Math.signum(leftSpeedB);
        rightSignB = (int)Math.signum(rightSpeedB);

        // Assign min speed with correct sign

        if (Math.abs(leftSpeed) < this.minSpeed) leftSpeed = this.minSpeed*leftSign;
        if (Math.abs(rightSpeed) < this.minSpeed) rightSpeed = this.minSpeed*rightSign;
        if (Math.abs(leftSpeedB) < this.minSpeed) leftSpeedB = this.minSpeed*leftSignB;
        if (Math.abs(rightSpeedB) < this.minSpeed) rightSpeedB = this.minSpeed*rightSignB;

        double [] normalizedSpeeds = {leftSpeed, rightSpeed,rightSpeedB,leftSpeedB};
        return  normalizedSpeeds;
    }

    //Function to find if the robot reached the desired target distance.
    //If desired distance is reached, it also checks if it is withing z angle tolerance.

    boolean distanceTargetReached( int xTicks, int yTicks){

        boolean onDistanceTarget = false;
        boolean onFinalTarget = false;
        if(xTicks == 0 && yTicks != 0){
            if(Math.abs(yTicks) - Math.abs(findYDisplacement()) <= DISTANCE_THRESHOLD){

                onDistanceTarget = true;

            }
        }else if(yTicks == 0 && xTicks != 0 ){
            if(Math.abs(xTicks) - Math.abs(findXDisplacement()) <= DISTANCE_THRESHOLD){

                onDistanceTarget = true;
            }

        }else{
            if((Math.abs(yTicks) - Math.abs(findYDisplacement()) <= DISTANCE_THRESHOLD) && (Math.abs(xTicks) - Math.abs(findXDisplacement()) <= DISTANCE_THRESHOLD)){

                onDistanceTarget = true;
            }

        }

        if(onDistanceTarget){

           if(!onTarget(getZAngleValue())){
                onFinalTarget = false;
            }
            else{
                onFinalTarget = true;
            }

        }

        return  onFinalTarget;
    }

    public void initializeZPIDCalculator(double kP, double kI, double kD, boolean debugFlag){

         this.zPIDCalculator = new NerdPIDCalculator("zPIDCalculator", kP, kI, kD);
         this.zPIDCalculator.setDebug(debugFlag);
    }

    public void initializeXPIDCalculator(double kP, double kI, double kD,boolean debugFlag){

        this.xPIDCalculator = new NerdPIDCalculator("xPIDCalculator", kP, kI, kD);
        this.xPIDCalculator.setDebug(debugFlag);
    }
    public void initializeYPIDCalculator(double kP, double kI, double kD,boolean debugFlag){

        this.yPIDCalculator = new NerdPIDCalculator("yPIDCalculator", kP, kI, kD);
        this.yPIDCalculator.setDebug(debugFlag);
    }

    public void initializeTurnPIDCalculator(double kP, double kI, double kD,boolean debugFlag){

        this.turnPIDCalculator = new NerdPIDCalculator("turnPIDCalculator", kP, kI, kD);
        this.turnPIDCalculator.setDebug(debugFlag);

    }

    public  double getZAngleValue(){

        return this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

  public  void setMinMaxSpeeds(double minSpeed, double maxSpeed){

        this.minSpeed = minSpeed;
        this.maxSpeed = maxSpeed;
    }

    public void setDebug(boolean debugFlag){
        this.debugFlag=debugFlag;
    }




}




