package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;


/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 */
@Autonomous(name= "AutoTemplate", group="Sky autonomous")
//@Disabled
public class AutoTemplate extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor backLeft     = null; //rear left
    private DcMotor backRight    = null; //rear right
    private DcMotor frontLeft    = null; //front left
    private DcMotor frontRight   = null; //front right

    private Servo servoRight        = null;
    private Servo servoLeft         = null;

    private final int encoderTicks = 1120;
    private final double wheelDiameter = 3.85827;//in inches


    OpenCvCamera phoneCam;

//    x, y = distance in x,y direction, angle = angle for rotation, power = motor power/speed
//    for now, use only one direction at a time
    public void move(double x, double y, double angle, double power) {
       backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

       backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       //y
       double distancePerRotationY = 3.1415 * wheelDiameter; //pi * diameter (inches) = circumference
       double rotationsY = y/distancePerRotationY; //distance / circumference (inches)
       int encoderTargetY = (int)(rotationsY*encoderTicks);

       //x
       double distancePerRotationX = 13.5; //distance per rotations is different than circumference when strafing (inches)
       double rotationsX = x/distancePerRotationX; //distance / circumference (inches)
       int encoderTargetX = (int)(rotationsX*encoderTicks);

       //angle
       double ticksPerRotation = 0;//measure how many ticks for a 360 rotation
       double rotationsA = angle/360;
       int encoderTargetA = (int)(rotationsA*ticksPerRotation);

       if(opModeIsActive()) {
         backLeft.setTargetPosition(encoderTargetY-encoderTargetX+encoderTargetA);
         backRight.setTargetPosition(encoderTargetY+encoderTargetX-encoderTargetA);
         frontLeft.setTargetPosition(encoderTargetY+encoderTargetX+encoderTargetA);
         frontRight.setTargetPosition(encoderTargetY-encoderTargetX-encoderTargetA);

         backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         backLeft.setPower(Math.abs(power));//childproof. must have always positive power
         backRight.setPower(Math.abs(power));
         frontLeft.setPower(Math.abs(power));
         frontRight.setPower(Math.abs(power));

         while(backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()) {
           //wait till motor finishes working
           telemetry.addData("Path", "Driving");
           telemetry.update();
         }

         backLeft.setPower(0);
         backRight.setPower(0);
         frontLeft.setPower(0);
         frontRight.setPower(0);

         telemetry.addData("Path", "Complete");
         telemetry.update();

         backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       }
   }


    @Override
    public void runOpMode() throws InterruptedException {
       backLeft        = hardwareMap.dcMotor.get("left_drive");
       backRight       = hardwareMap.dcMotor.get("right_drive");
       frontLeft       = hardwareMap.dcMotor.get("front_left");
       frontRight      = hardwareMap.dcMotor.get("front_right");
       servoLeft       = hardwareMap.servo.get("servoLeft");
       servoRight      = hardwareMap.servo.get("servoRight");

       backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       backRight.setDirection(DcMotorSimple.Direction.FORWARD);
       frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        while (opModeIsActive())
        {

            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }
    }
}
