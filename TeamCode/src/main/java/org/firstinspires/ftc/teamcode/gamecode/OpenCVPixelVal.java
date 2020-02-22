/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.gamecode;

import android.provider.ContactsContract;
import android.service.notification.NotificationListenerService;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Rectangle;
import org.opencv.core.Point;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Range;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.robots.Joules;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.List;

/**
 * In this sample, we demonstrate how to use the {@link OpenCvPipeline#onViewportTapped()}
 * callback to switch which stage of a pipeline is rendered to the viewport for debugging
 * purposes. We also show how to get data from the pipeline to your OpMode.
 */
@Autonomous

public class OpenCVPixelVal extends LinearOpMode
{
    OpenCvCamera phoneCam;
    StageSwitchingPipeline stageSwitchingPipeline;
//    Joules joules = new Joules();
    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCameraExample} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        stageSwitchingPipeline = new StageSwitchingPipeline();
        phoneCam.setPipeline(stageSwitchingPipeline);
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();



        while (opModeIsActive())
        {
            telemetry.addData("Num contours found", stageSwitchingPipeline.getNumContoursFound());
            telemetry.addData("Mean", stageSwitchingPipeline.getMean());
            telemetry.addData("Mean RectOneRed", stageSwitchingPipeline.getRectOneRed());
            telemetry.addData("Mean RectTwoGreen", stageSwitchingPipeline.getRectTwoGreen());
            telemetry.addData("Mean RectTwoGreen", stageSwitchingPipeline.getRectThreeBlue());


            telemetry.update();
            sleep(100);
        if (opModeIsActive()) {
            if (stageSwitchingPipeline.getRectOneRed() < stageSwitchingPipeline.getRectTwoGreen() && stageSwitchingPipeline.getRectOneRed() < stageSwitchingPipeline.getRectThreeBlue()) {
                telemetry.addData("Skystone", "Left");
                //joules.StrafeRight(0.3);
            }

            else if (stageSwitchingPipeline.getRectThreeBlue() < stageSwitchingPipeline.getRectOneRed() && stageSwitchingPipeline.getRectThreeBlue() < stageSwitchingPipeline.getRectTwoGreen()) {
                telemetry.addData("Skystone", "Right");
                //joules.StrafeLeft(0.3);
            }

            else if (stageSwitchingPipeline.getRectTwoGreen() < stageSwitchingPipeline.getRectOneRed() && stageSwitchingPipeline.getRectTwoGreen() < stageSwitchingPipeline.getRectThreeBlue()) {
                telemetry.addData("Skystone", "Center");
                //go to rest of program
            }
        }
        }

    }


    /*
     * With this pipeline, we demonstrate how to change which stage of
     * is rendered to the viewport when the viewport is tapped. This is
     * particularly useful during pipeline development. We also show how
     * to get data from the pipeline to your OpMode.
     */
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat RectOneRed = new Mat();
        Mat RectTwoGreen = new Mat();
        Mat RectThreeBlue = new Mat();
        Mat YCrCB = new Mat();
        final int x1 = 5;
        final int x2 = 155;
        final int x3 = 165;
        final int x4 = 315;
        final int x5 = 325;
        final int x6 = 475;

        final int y1 = 500;
        final int y2 = 600;
        List<MatOfPoint> contoursList = new ArrayList<>();
        int numContoursFound;
        double mean;

        enum Stage
        {   IMAGE,
            RECT1,
            RECT2,
            RECT3,
        }

        private Stage stageToRenderToViewport = Stage.IMAGE;

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

        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, YCrCB, Imgproc.COLOR_RGB2YCrCb);
             Core.extractChannel(YCrCB, YCrCB, 0);

           RectOneRed = YCrCB.submat(y1,y2,x1,x2);
           RectTwoGreen = YCrCB.submat(y1,y2,x3,x4);
           RectThreeBlue = YCrCB.submat(y1,y2,x5,x6);




           Imgproc.rectangle(
                    YCrCB,                    //Matrix obj of the image
                    new Point(x1, y1),        //p1
                    new Point(x2, y2),       //p2
                    new Scalar(255, 0, 0),     //Red
                    5                          //Thickness of the line
            );

           Imgproc.rectangle (
                    YCrCB,                    //Matrix obj of the image
                    new Point(x3, y1),        //p1
                    new Point(x4, y2),       //p2
                    new Scalar(0, 255, 0),     //Green
                    5                          //Thickness of the line
            );
           Imgproc.rectangle (
                    YCrCB,                    //Matrix obj of the image
                    new Point(x5, y1),        //p1
                    new Point(x6, y2),       //p2
                    new Scalar(0, 0, 255),     //Blue
                    5                          //Thickness of the line
            );




            switch (stageToRenderToViewport)
           {

               case IMAGE:
               {
                   return YCrCB;
               }
               case RECT1:{
                   return RectOneRed;
               }
               case RECT2:{
                   return RectTwoGreen;
               }
               case RECT3:{
                   return RectThreeBlue;
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

        public double getMean(){
         return Core.mean(YCrCB).val[0];
        }

        public double getRectOneRed(){
            return Core.mean(RectOneRed).val[0];
        }

        public double getRectTwoGreen(){
            return Core.mean(RectTwoGreen).val[0];
        }

        public double getRectThreeBlue(){
            return Core.mean(RectThreeBlue).val[0];
        }


    }


}