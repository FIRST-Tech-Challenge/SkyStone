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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



//import lines go here. This is just for the program and not for the robot.

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="NerdPIDDriveForQuad", group="Linear Opmode")
//@Disabled
public class NerdPIDriveForQuad extends LinearOpMode {

   private DcMotor leftMotor;
   private DcMotor rightMotor;
   private DcMotor leftMotorB;
   private DcMotor rightMotorB;
   private ElapsedTime runtime = new ElapsedTime();
   private BNO055IMU imu = null;   // Gyro device


    //Initial Speed for Robot to run

    private  double speed = 0.4;
    private double maxSpeed = 0.6;


    private double ticksPerRotation = 560.0;

    private double wheelDiameter = 4.0;

    // Use these for Competition Robot:
//
//    NerdPIDCalculator zPIDCalculator = new NerdPIDCalculator("zPID", 0.008, 0.00, 1.6);
//    NerdPIDCalculator turnPIDCalculator = new NerdPIDCalculator("TurnPID", 0.009, 0.000, 0.9);
//    NerdPIDCalculator xPIDCalculator = new NerdPIDCalculator("xPID", 0.002, 0.0, 1.1);
//    NerdPIDCalculator yPIDCalculator = new NerdPIDCalculator("yPID", 0.0, 0.0, 0.0);
//    private double minSpeed = 0.13;

    // Use these for Quad Robot:

    private double minSpeed = 0.1;
    NerdPIDCalculator zPIDCalculator = new NerdPIDCalculator("zPID", 0.036, 0.08, 0.0);
    NerdPIDCalculator turnPIDCalculator = new NerdPIDCalculator("TurnPID", 0.024, 0.000, 1.85);
    NerdPIDCalculator xPIDCalculator = new NerdPIDCalculator("xPID", 0.0005, 0.005, 0.5);
    NerdPIDCalculator yPIDCalculator = new NerdPIDCalculator("yPID", 0.0005, 0.005, 0.5);




    static final double HEADING_THRESHOLD = 1;


    @Override
    public void runOpMode() {

        initializeHardware();

        waitForStart();


         RobotLog.d("PID - PIDrive - Starting Straight Drive");


        nerdPidDrive( speed, 0.0, 25, 0.0);

          sleep(1000);
          RobotLog.d("PID - Angle After PIDDrive : %f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

        nerdPidDrive( speed, 25.0, 0.0, 0.0);
        sleep(1000);

        nerdPidDrive( speed, 0.0, -25, 0.0);
        sleep(1000);

        nerdPidDrive( speed, -25.0, 25.0, 0.0);

        RobotLog.d("PID - Angle After PIDDrive : %f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        RobotLog.d("PID - PIDTurn - Starting Straight Drive");

        sleep(1000);
        nerdPidTurn(0.2, 90.0, false);
        RobotLog.d("PID - FinalAngle After PIDTurn : %f", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);



        while (opModeIsActive())  {

        }


    }


    public void nerdPidDrive(double speed, double xDistance,double yDistance, double zAngleToMaintain) {

        final String funcName = "nerdPidDrive";

        double leftSpeed;
        double rightSpeed;
        double rightSpeedB;
        double leftSpeedB;

        int xTicks,yTicks;

        double zpid, xpid, ypid;
        double [] motorPowers;



        runtime.reset();
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

        xPIDCalculator.reset();
        yPIDCalculator.reset();
        zPIDCalculator.reset();

        motorsResetAndRunUsingEncoders();


        xTicks = xDistance != 0.0 ? (int)inchesToTicksForQuadStraightDrive(wheelDiameter,xDistance, 45.0): 0;
        yTicks = yDistance != 0.0 ? (int)inchesToTicksForQuadStraightDrive(wheelDiameter, yDistance, 45.0): 0;

        RobotLog.d ("PID - xTicks = %d, yTicks = %d , Angle %f", xTicks, yTicks, zAngleToMaintain);

        xPIDCalculator.setTarget(xTicks,false,0.0);
        yPIDCalculator.setTarget(yTicks,false,findYDisplacement());
        zPIDCalculator.setTarget(zAngleToMaintain,false,0.0);

        RobotLog.d ("PID - xTicks = %d, yTicks = %d , Angle %f", xTicks, yTicks, zAngleToMaintain);



        RobotLog.d ("WhileCondition  xDisp - %f,yDisp = %f , onTarget %b", Math.abs(findXDisplacement()), Math.abs(findXDisplacement()),
                onTarget((imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)));

    while ( opModeIsActive() && ( !distanceTargetReached(xTicks,yTicks) )) {
      //while (opModeIsActive() && runtime.seconds() < 3 ) {
            zpid = zPIDCalculator.getOutput(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle,1);
            xpid = xPIDCalculator.getOutput(findXDisplacement(),2); // Device type 1 = gyro, 2= other
            ypid = yPIDCalculator.getOutput(findYDisplacement(),2);
            RobotLog.d("XPID - %f, YPID - %f , ZPID - %f", xpid, ypid, zpid);


                    leftSpeed = ypid - zpid + xpid;
                    rightSpeed = ypid + zpid - xpid;
                    leftSpeedB = ypid - zpid - xpid;
                    rightSpeedB = ypid + zpid + xpid;

            RobotLog.d("PID - OpMode - Speeds before Normalizing : %s |leftSpeed | rightSpeed | leftSpeedB | rightSpeedB ", funcName);

            RobotLog.d("PID - OpMode - Speeds before Normalizing : %s |%f|%f|%f|%f ", funcName,leftSpeed,rightSpeed,leftSpeedB,rightSpeedB);


           motorPowers = normalizeSpeedsForMinMaxValues(leftSpeed,rightSpeed,rightSpeedB,leftSpeedB);

           leftMotor.setPower(motorPowers[0]);
           rightMotor.setPower(motorPowers[1]);
           rightMotorB.setPower(motorPowers[2]);
           leftMotorB.setPower(motorPowers[3]);

           //////////////////////////////////////////////////////////////////////////////////

//            telemetry.addData("leftMotorTicks", leftMotor.getCurrentPosition());
//            telemetry.addData("rigthMotorTicks", rightMotor.getCurrentPosition());
//            telemetry.addData("leftMotorBTicks", leftMotorB.getCurrentPosition());
//            telemetry.addData("rigthMotorBTicks", rightMotorB.getCurrentPosition());
//
//            telemetry.update();
            RobotLog.d("PID - OpMode - Speeds after Normalizing : %s |leftSpeed | rightSpeed | leftSpeedB | rightSpeedB ", funcName);
            RobotLog.d("PID - OpMode - Speeds after Normalizing : %s |%f|%f|%f|%f ", funcName,motorPowers[0],motorPowers[1],motorPowers[2],motorPowers[3]);

        }
//        while (opModeIsActive() && (!onTarget(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle))){
//
//            zpid = zPIDCalculator.getOutput(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle,1);
//            leftSpeed = minSpeed - zpid;
//            rightSpeed = minSpeed + zpid;
//            leftSpeedB = minSpeed - zpid;
//            rightSpeedB = minSpeed + zpid;
//
//            leftMotor.setPower(leftSpeed);
//            rightMotor.setPower(rightSpeed);
//            rightMotorB.setPower(rightSpeedB);
//            leftMotorB.setPower(leftSpeedB);
//
//        }

        brakeMotorsAndStop();

    }

    public void nerdPidTurn(double turnspeed, double targetAngle, boolean isRelativeAngle) {

        final String funcName = "PIDTurn";

        Orientation angles;
        double pidvalue;
        double [] motorPowers;


        double leftSpeed=0, rightSpeed=0, rightSpeedB=0, leftSpeedB=0;

        runtime.reset();

        turnPIDCalculator.setTarget(targetAngle, isRelativeAngle, 0.0);

        motorsSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

          while (opModeIsActive() && (!onTarget(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle))) {
                    //(averageMotorPower(leftMotorB, rightMotorB, rightMotor, leftMotor) > 0.1   ))) { //&& leftMotor.isBusy() && rightMotor.isBusy()) {
       // while (opModeIsActive() && runtime.seconds() < 5 ) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                pidvalue = turnPIDCalculator.getOutput(angles.firstAngle, 1);


                leftSpeedB =  -pidvalue;
                rightSpeedB =  pidvalue;
                leftSpeed =  -pidvalue;
                rightSpeed = pidvalue;

                RobotLog.d("PID - OpMode - Speeds before Normalizing : %s |leftSpeed | rightSpeed | leftSpeedB | rightSpeedB ", funcName);

                RobotLog.d("PID - OpMode - Speeds before Normalizing : %s |%f|%f|%f|%f", funcName,leftSpeed,rightSpeed,leftSpeedB,rightSpeedB);


                motorPowers = normalizeSpeedsForMinMaxValues(leftSpeed,rightSpeed,rightSpeedB,leftSpeedB);


                leftMotor.setPower(motorPowers[0]);
                rightMotor.setPower(motorPowers[1]);
                rightMotorB.setPower(motorPowers[2]);
                leftMotorB.setPower(motorPowers[3]);

                //////////////////////////////////////////////////////////////////////////////////
                RobotLog.d("PID - OpMode - %s : current Angle |  pidValue ", funcName);
                RobotLog.d ("PID - OPMode - %s  | %f |%f ", funcName,imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, pidvalue);
                telemetry.addData("Angle", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
                RobotLog.d("PID - OpMode - Speeds after Normalizing : %s |leftSpeed | rightSpeed | leftSpeedB | rightSpeedB ", funcName);
                RobotLog.d("PID - OpMode - Speeds after Normalizing : %s |%f|%f|%f|%f ", funcName,leftSpeed,rightSpeed,leftSpeedB,rightSpeedB);


            }

            brakeMotorsAndStop();
    }


    public int inchesToTicksForQuadStraightDrive(double wheelDiameter, double straightDistance, double wheelMountAngle){
        int ticks;
        double circum = wheelDiameter * 3.14;
        double wheelDistanceToTravel = straightDistance/ Math.cos(Math.toRadians(wheelMountAngle));
        RobotLog.d("inchesToTicksForQuadStraightDrive - wheelDistanceToTravel - %f, straightDistance - %f",wheelDistanceToTravel,straightDistance );
        double numberOfWheelRotations = wheelDistanceToTravel/circum;

        RobotLog.d("inchesToTicksForQuadStraightDrive - numberOfWheelRotations - %f",numberOfWheelRotations );

        ticks =  (int)Math.round(ticksPerRotation * numberOfWheelRotations);
        RobotLog.d("inchesToTicksForQuadStraightDrive - ticks - %d",ticks );

        return ticks;
    }

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

double findXDisplacement(){

        return (leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition()
                -leftMotorB.getCurrentPosition() + rightMotorB.getCurrentPosition())/4.0;

}

double findYDisplacement(){

    return (this.leftMotor.getCurrentPosition() + this.rightMotor.getCurrentPosition()
            + this.leftMotorB.getCurrentPosition() + this.rightMotorB.getCurrentPosition())/4.0;

}

public void initializeHardware(){

        //Initialize Motors

    this.leftMotor = hardwareMap.dcMotor.get("Front_Left_Motor");
    this.rightMotor = hardwareMap.dcMotor.get("Front_Right_Motor");
    this.leftMotorB = hardwareMap.dcMotor.get("Rear_Left_Motor");
    this.rightMotorB = hardwareMap.dcMotor.get("Rear_Right_Motor");

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
    this.imu = hardwareMap.get(BNO055IMU.class, "imu");
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

//    this.leftMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
//    this.leftMotorB.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
//    this.rightMotor.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
//    this.rightMotorB.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);



    this.leftMotor.setTargetPosition(this.leftMotor.getCurrentPosition());
    this.rightMotor.setTargetPosition(this.rightMotor.getCurrentPosition());
    this.leftMotorB.setTargetPosition(this.leftMotorB.getCurrentPosition());
    this.rightMotorB.setTargetPosition(this.rightMotorB.getCurrentPosition());

    this.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.leftMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    this.rightMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//    this.leftMotor.setPower(0);
//    this.rightMotor.setPower(0);
//    this.leftMotorB.setPower(0);
//    this.rightMotorB.setPower(0);
}

    public void motorsSetMode(DcMotor.RunMode runMode){

        this.leftMotor.setMode(runMode);
        this.rightMotor.setMode(runMode);
        this.rightMotorB.setMode(runMode);
        this.leftMotorB.setMode(runMode);

    }

    public double[] normalizeSpeedsForMinMaxValues(double leftSpeed, double rightSpeed, double rightSpeedB, double leftSpeedB){

        double max = Math.max(Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed)), Math.max(Math.abs(leftSpeedB), Math.abs(rightSpeedB)));
        if (max > maxSpeed) {
            leftSpeed = (leftSpeed * this.maxSpeed)/max;
            rightSpeed =(rightSpeed * this.maxSpeed)/ max;
            leftSpeedB = (leftSpeedB * this.maxSpeed)/max;
            rightSpeedB = (rightSpeedB * this.maxSpeed)/max;
        }


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

    boolean distanceTargetReached( int xTicks, int yTicks){

        if(xTicks == 0 && yTicks != 0){
            if(Math.abs(yTicks) - Math.abs(findYDisplacement()) <= 25){

                telemetry.addData("yTicks", yTicks);
                telemetry.addData("findYDisplacement", findYDisplacement());

                telemetry.update();
                return true;

            }
        }else if(yTicks == 0 && xTicks != 0 ){
            if(Math.abs(xTicks) - Math.abs(findXDisplacement()) <= 25){
                telemetry.addData("xTicks", xTicks);
                telemetry.addData("findXDisplacement", findXDisplacement());

                telemetry.update();
                return true;
            }

        }else{
            if((Math.abs(yTicks) - Math.abs(findYDisplacement()) <= 25) && (Math.abs(xTicks) - Math.abs(findYDisplacement()) <= 25)){
                telemetry.addData("yTicks", yTicks);
                telemetry.addData("findYDisplacement", findYDisplacement());
                telemetry.addData("xTicks", xTicks);
                telemetry.addData("findXDisplacement", findXDisplacement());

                telemetry.update();

                return true;
            }

        }
        return  false;
    }

}




