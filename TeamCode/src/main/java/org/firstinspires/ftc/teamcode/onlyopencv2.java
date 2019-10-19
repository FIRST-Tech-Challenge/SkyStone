package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
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
 *
 */
@Autonomous(name= "onlyopencv2", group="Sky autonomous")
//@Disabled
public class onlyopencv2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //    private DcMotor backLeft     = null; //rear left
//    private DcMotor backRight    = null; //rear right
//    private DcMotor frontLeft    = null; //front left
//    private DcMotor frontRight   = null; //front right
//
//    private Servo   servo        = null;
    private final int encoderTicks = 1120;
    private final double wheelDiameter = 3.85827;//in inches
    public static double valMid = -1;
    public static double valLeft = -1;
    public static double valRight = -1;

    public static float[] midPos = {1f/2f, 5f/8f};//0 = col, 1 = row
    public static float[] leftPos = {1f/4f, 5f/8f};
    public static float[] rightPos = {3f/4f, 5f/8f};

    public static double rows = 0;
    public static double cols = 0;


    OpenCvCamera phoneCam;
    StageSwitchingPipeline stageSwitchingPipeline;

    //x, y = distance in x,y direction, angle = angle for rotation, power = motor power/speed
    //x must equal y if both are nonzero.
//    public void move(double x, double y, double angle, double power) {
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//
//          if(x == y){
//              double h = Math.sqrt(x*x+y*y);
//              x=h;
//              y=h;
//          }

//        //y
//        double distancePerRotationY = 3.1415 * wheelDiameter; //pi * diameter (inches)
//        double rotationsY = y/distancePerRotationY; //distance / circumference (inches)
//        int encoderTargetY = (int)(rotationsY*encoderTicks);
//
//        //x
//        double distancePerRotationX = 13.5; //distance per rotations is different than circumference when strafing (inches)
//        double rotationsX = x/distancePerRotationX; //distance / circumference (inches)
//        int encoderTargetX = (int)(rotationsX*encoderTicks);
//
//        //angle
//        double ticksPerRotation = 0;//measure how many ticks for a 360 rotation
//        double rotationsA = angle/360;
//        int encoderTargetA = (int)(rotationsA*ticksPerRotation);
//
//        if(opModeIsActive()) {
             // if(x==y)
               //
                //
//            backLeft.setTargetPosition(encoderTargetY-encoderTargetX+encoderTargetA);
//            backRight.setTargetPosition(encoderTargetY+encoderTargetX-encoderTargetA);
//            frontLeft.setTargetPosition(encoderTargetY+encoderTargetX+encoderTargetA);
//            frontRight.setTargetPosition(encoderTargetY-encoderTargetX-encoderTargetA);
//
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            backLeft.setPower(Math.abs(power));//childproof. must have always positive power
//            backRight.setPower(Math.abs(power));
//            frontLeft.setPower(Math.abs(power));
//            frontRight.setPower(Math.abs(power));
//
//            while(backLeft.isBusy() || backRight.isBusy() || frontLeft.isBusy() || frontRight.isBusy()) {
//                //wait till motor finishes working
//                telemetry.addData("Path", "Driving "+distance+" inches");
//                telemetry.update();
//            }
//
//            backLeft.setPower(0);
//            backRight.setPower(0);
//            frontLeft.setPower(0);
//            frontRight.setPower(0);
//
//            telemetry.addData("Path", "Complete");
//            telemetry.update();
//
//            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Running");
        telemetry.update();

//        backLeft        = hardwareMap.dcMotor.get("left_drive");
//        backRight       = hardwareMap.dcMotor.get("right_drive");
//        frontLeft       = hardwareMap.dcMotor.get("front_left");
//        frontRight      = hardwareMap.dcMotor.get("front_right");
//        servo           = hardwareMap.servo.get("servo");

//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        //phoneCam.setPipeline(new SamplePipeline());//add rectangle
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);//display on RC

        waitForStart();
        runtime.reset();
        while (opModeIsActive())
        {

            //telemetry.addData("Num contours found", stageSwitchingPipeline.getNumContoursFound());
            telemetry.addData("Values", valLeft+";"+valMid+";"+valRight);
            telemetry.addData("Rows", rows);
            telemetry.addData("Cols", cols);

            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }
    }

    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            RAW_IMAGE,//displays raw view
            PROCESSED,
        }

        private Stage stageToRenderToViewport = Stage.RAW_IMAGE;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            rows = input.rows();
            cols = input.cols();

            /*
             * This pipeline finds the contours of yellow blobs such as the Gold Mineral
             * from the Rover Ruckus game.
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb


            System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
            Mat template;
            String filePath="C:\\Users\\mesutpiskin\\Desktop\\Object Detection\\Template Matching\\Sample Image\\";
            //Load image file
            template=Imgcodecs.imread(filePath+"pic.png");

            Mat outputImage=new Mat();
            int machMethod=Imgproc.TM_CCOEFF;
            //Template matching method
            Imgproc.matchTemplate(input, template, outputImage, machMethod);


            Core.MinMaxLocResult mmr = Core.minMaxLoc(outputImage);
            Point matchLoc=mmr.maxLoc;
            //Draw rectangle on result image
            Imgproc.rectangle(input, matchLoc, new Point(matchLoc.x + template.cols(),
                    matchLoc.y + template.rows()), new Scalar(255, 255, 255));

            Imgcodecs.imwrite(filePath+"pic.png", input);
            System.out.println("Complated.");



            switch (stageToRenderToViewport)
            {

                case PROCESSED:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}