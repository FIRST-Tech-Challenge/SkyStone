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
@Autonomous(name= "opencvtest1", group="Sky autonomous")
//@Disabled
public class onlyopencv extends LinearOpMode {
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

    public static float[] midPos = {1f/2f, 5f/8f};
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
            telemetry.addData("Middle", valMid);
            telemetry.addData("Left", valLeft);
            telemetry.addData("Right", valRight);
            telemetry.addData("Rows", rows);
            telemetry.addData("Cols", cols);

            telemetry.update();
            sleep(100);
            //call movement functions
//            strafe(0.4, 200);
//            moveDistance(0.4, 700);

        }


    }

    //detection, get pipeline data to opmode
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        int numContoursFound;

        enum Stage
        {
            YCbCr_CHAN2,//color difference. greyscale
            THRESHOLD,//b&w
            detection,//includes outlines
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.YCbCr_CHAN2;
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
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 102, 255, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();//sets size according to number of contours/outlines
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixMid = thresholdMat.get((int)(input.rows()* midPos[0]), (int)(input.cols()* midPos[1]));//gets value at circle
            valMid = pixMid[0];

            double[] pixLeft = thresholdMat.get((int)(input.rows()* leftPos[0]), (int)(input.cols()* leftPos[1]));//gets value at circle
            valLeft = pixLeft[0];

            double[] pixRight = thresholdMat.get((int)(input.rows()* rightPos[0]), (int)(input.cols()* rightPos[1]));//gets value at circle
            valRight = pixRight[0];


            //create three points
            Point point1 = new Point((int)(input.cols()* midPos[0]), (int)(input.rows()* midPos[1]));
            Point point2 = new Point((int)(input.cols()* leftPos[0]), (int)(input.rows()* leftPos[1]));
            Point point3 = new Point((int)(input.cols()* rightPos[0]), (int)(input.rows()* rightPos[1]));

            //draw circles on those points
            Imgproc.circle(all, point1,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, point2,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, point3,5, new Scalar( 255, 0, 0 ),1 );//draws circle


            MatOfPoint2f approxCurve = new MatOfPoint2f();
            //For each contour found
            for (int i=0; i<contoursList.size(); i++)
            {
                //Convert contours(i) from MatOfPoint to MatOfPoint2f
                MatOfPoint2f contour2f = new MatOfPoint2f( contoursList.get(i).toArray() );
                //Processing on mMOP2f1 which is in type MatOfPoint2f
                double approxDistance = Imgproc.arcLength(contour2f, true)*0.02;
                Imgproc.approxPolyDP(contour2f, approxCurve, approxDistance, true);

                //Convert back to MatOfPoint
                MatOfPoint points = new MatOfPoint(approxCurve.toArray() );
                // Get bounding rect of contour
                Rect rect = Imgproc.boundingRect(points);

                if((rect.contains(point1) || rect.contains(point2) || rect.contains(point3)) && (valMid == 0 || valLeft == 0 || valRight == 0)) {
                    Imgproc.rectangle(all,
                            new Point(rect.x, rect.y),
                            new Point(rect.x + rect.width, rect.y + rect.height),
                            new Scalar(255, 0, 0, 255), 3);
                    break;
                }

            }

            Imgproc.rectangle(
                    all,
                    new Point(
                            input.cols()/8,
                            input.rows()*4.5/8),
                    new Point(
                            input.cols()*(2.9f/8f),
                            input.rows()*(5.5f/8f)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(
                    all,
                    new Point(
                            input.cols()*(3.1/8),
                            input.rows()*4.5/8),
                    new Point(
                            input.cols()*(4.9f/8f),
                            input.rows()*(5.5f/8f)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(
                    all,
                    new Point(
                            input.cols()*(5.1/8),
                            input.rows()*4.5/8),
                    new Point(
                            input.cols()*(7f/8f),
                            input.rows()*(5.5f/8f)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case YCbCr_CHAN2:
                {
                    return yCbCrChan2Mat;
                }

                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
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

        public int getNumContoursFound()
        {
            return numContoursFound;
        }
    }
}