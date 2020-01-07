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

package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * Created by 12090 STEM Punk
 */
public abstract class OmniAutoFullToF extends OmniAutoClass
{
    // Coordinates for the vision pipeline to be overriden in the alliance classes.
    protected Point sub1PointA;
    protected Point sub1PointB;
    protected Point sub2PointA;
    protected Point sub2PointB;
    protected Point sub3PointA;
    protected Point sub3PointB;

    OpenCvCamera phoneCam;
    protected int stoneGrabTime = 1150;
    public static int position = 0;
    protected double attackAngle1 = 40;
    protected double sideDistance1 = 50.3;
    protected int flyTime1 = 1300;
    protected int flyBackTime1 = 1200;

	protected double attackAngle2 = 40.0;
    protected double sideDistance2 = 101.3;
	protected int flyTime2 = 1300;
	protected int flyBackTime2 = 200;
	
    protected double baseAngle;
    protected double fudgeAngle = 0;
    protected boolean progressActivities = false;
    protected boolean runLift = true;
    protected boolean secondSkystone = true;

    public abstract void setSkystoneValues(int position);
    public abstract void setVisionPoints();

    @Override
    public void runOpMode()
    {
        int timeout = 0;
		double maxSpeed = 1.0;
		double slowSpeed = 0.3;
		double precisionSpeed = 0.05;
		double foundationRotateSpeed = 0.6;
		double rotateSpeed = 0.3;
		double slowSpin = 0.1;
		double precisionSpin = 0.05;
		int stonePosition = 1;

		// Setup the data needed for the vision pipeline
        setVisionPoints();

        // Error to consider distance from wall a success
		// 1 cm
		double standardDistanceError = 1.0;
		// 0.5 cm
		double precisionDistanceError = 0.5;

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new OmniAutoFullToF.SamplePipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        setupRobotParameters(4.0, 19.2);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        stonePosition = position;
        telemetry.addData("Stone Position: ", stonePosition);
        telemetry.update();

		// Stop the image pipeline.
		phoneCam.stopStreaming();

		// Start moving intake out, should be done by the time driving is done.
        robot.startExtendingIntake();

        // This sets side specific values based on red or blue
        setSkystoneValues(stonePosition);

        // Drive out from the starting position, 10cm from the skystones
        distanceFromWall(HardwareOmnibot.RobotSide.BACK, 54.0, maxSpeed, standardDistanceError, 5000, progressActivities);

        // Drive from the side wall to the collection identified stone position.

        // Rotate the robot to collection angle.
        rotateRobotToAngle(rotateSpeed, baseAngle + attackAngle1, 2000, progressActivities);

        // Make sure the intake is out.
        robot.resetReads();
        double endTime = timer.milliseconds() + 1000;
        while (!robot.intakeExtended() && (timer.milliseconds() < endTime) && (!isStopRequested())) {
            robot.resetReads();
        }
        robot.moveLift(HardwareOmnibot.LiftPosition.STOWED);

        // Start the intake to collect.
        robot.startIntake(false);

        // Drive forward to collect the skystone and drive back.
        driveAtHeadingForTime(slowSpeed, precisionSpin, baseAngle + 90 + attackAngle1, baseAngle + attackAngle1, stoneGrabTime, true, progressActivities);
        driveAtHeadingForTime(slowSpeed, precisionSpin, baseAngle + 270 + attackAngle1, baseAngle + attackAngle1, stoneGrabTime, true, progressActivities);

        // Stop the intake
        robot.stopIntake();

        // Rotate to running angle to go to other side of the bridge.
        rotateRobotToAngle(rotateSpeed, baseAngle + 90.0 + fudgeAngle, 2000, progressActivities);

        // Move robot to center of lane before launching.  Lane defined as
        // skybridge 48 inches and robot width 18 inches, with our robot width
        // that is 24 inches to 42 inches, leaving 6 inches on either side.

        // Fly to the other side.  Do not put the brakes on, allow the distance
        // from wall function take over.
        // We could turn off encoders to drive faster.
        driveAtHeadingForTime(maxSpeed, slowSpin, baseAngle + 0.0 + fudgeAngle, baseAngle + 90.0 + fudgeAngle, flyTime1, false, progressActivities);

        // Get to foundation midpoint.
        distanceFromWall(HardwareOmnibot.RobotSide.BACK, 40.0, maxSpeed, standardDistanceError, 5000, progressActivities);

        // Start delivering Stone
        if(runLift) {
            robot.resetReads();
            robot.liftTargetHeight = HardwareOmnibot.LiftPosition.STONE_AUTO;
            robot.startStoneStacking();
        }

        // Rotate to foundation grabbing angle.
        // Not sure why this doesn't need base angle
        rotateRobotToAngle(rotateSpeed, 180.0 + fudgeAngle, 2000, true);

        // Back the robot up to the foundation
        grabFoundation(5000);

        // Move the foundation to parallel back wall.
        // Not sure why this doesn't need base angle
        if(baseAngle == 0) {
            driveAtHeadingForTime(maxSpeed, slowSpin, 260.0, 170.0, 350, false, true);
            driveAtHeadingForTime(maxSpeed, slowSpin, 250.0, 160.0, 350, false, true);
        } else {
            driveAtHeadingForTime(maxSpeed, slowSpin, 290.0+fudgeAngle, 200.0+fudgeAngle, 450, false, true);
            driveAtHeadingForTime(maxSpeed, slowSpin, 300.0+fudgeAngle, 210.0+fudgeAngle, 450, false, true);
        }

        // Rotate to parallel back wall.
        rotateRobotToAngle(rotateSpeed, baseAngle + 90.0 + fudgeAngle, 2000, true);

        // Drive the foundation into the back wall.
        driveAtHeadingForTime(maxSpeed, precisionSpin, baseAngle + 0.0, baseAngle + 90.0, 500, true, true);

        // Release the foundation
        moveFingers(true, true);

        // Perform the whole stone placement here for now.  Place at level 2 to make sure it doesn't get caught up.
        if(runLift) {
            endTime = timer.milliseconds() + 10000;
            while ((robot.stackStone != HardwareOmnibot.StackActivities.IDLE) && (timer.milliseconds() < endTime) && (!isStopRequested())) {
                robot.resetReads();
                performRobotActivities();
            }
        }

        // Get to running distance from the wall before placing.
        // Move robot to center of lane before launching.  Lane defined as
        // skybridge 48 inches and robot width 18 inches, with our robot width
        // that is 24 inches to 42 inches, leaving 6 inches on either side.

        if(secondSkystone) {
            // Fly back to the other side to collect second stone.
            driveAtHeadingForTime(maxSpeed, slowSpin, baseAngle + 180.0, baseAngle + 90.0, flyBackTime1, true, false);

            // Rotate the robot to line up to collect.
            // Not sure why we don't use base angle here.
            rotateRobotToAngle(rotateSpeed, 0.0, 2000, false);

            // Drive out to 10cm from the skystones
            distanceFromWall(HardwareOmnibot.RobotSide.BACK, 54.0, maxSpeed, standardDistanceError, 5000, false);

            // Drive from the side wall to the collection identified stone position.

            // Rotate the robot to collection angle.
            rotateRobotToAngle(rotateSpeed, baseAngle + attackAngle2, 2000, false);

            // Start the intake to collect.
            robot.startIntake(false);

            // Drive forward to collect the skystone.
            driveAtHeadingForTime(slowSpeed, precisionSpin, baseAngle + 90 + attackAngle2, baseAngle + attackAngle2, stoneGrabTime, true, false);

            // Rotate to running angle to go to other side of the bridge.
            rotateRobotToAngle(rotateSpeed, baseAngle + 90.0, 2000, false);

            // Stop the intake
            robot.stopIntake();

            // Move robot to center of lane before launching.  Lane defined as
            // skybridge 48 inches and robot width 18 inches, with our robot width
            // that is 24 inches to 42 inches, leaving 6 inches on either side.

            // Fly to the other side.  Do not put the brakes on, allow the distance
            // from wall function take over.
            driveAtHeadingForTime(maxSpeed, slowSpin, baseAngle + 0.0, baseAngle + 90.0, flyTime2, true, false);
            driveAtHeadingForTime(maxSpeed, slowSpin, baseAngle + 180.0, baseAngle + 90.0, flyBackTime2, true, false);
        } else {
            driveAtHeadingForTime(maxSpeed, slowSpin, baseAngle + 180.0, baseAngle + 90.0, flyBackTime2, true, false);
        }
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat YCrCb = new Mat();
        private Mat Cb = new Mat();
        private Mat subMat1;
        private Mat subMat2;
        private Mat subMat3;
        private int max;
        private int avg1;
        private int avg2;
        private int avg3;
        private Point skystone = new Point();

        @Override
        public Mat processFrame(Mat input)
        {
            // Convert the image from RGB to YCrCb
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

            // Extract the Cb channel from the image
            Core.extractChannel(YCrCb, Cb, 2);

            // The the sample areas fromt the Cb channel
            subMat1 = Cb.submat(new Rect(sub1PointA, sub1PointB));
            subMat2 = Cb.submat(new Rect(sub2PointA, sub2PointB));
            subMat3 = Cb.submat(new Rect(sub3PointA, sub3PointB));

            // Average the sample areas
            avg1 = (int)Core.mean(subMat1).val[0]; // Stone4
            avg2 = (int)Core.mean(subMat2).val[0]; // Stone5
            avg3 = (int)Core.mean(subMat3).val[0]; // Stone6

            // Draw rectangles around the sample areas
            Imgproc.rectangle(input, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, sub2PointA, sub2PointB, new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, sub3PointA, sub3PointB, new Scalar(0, 0, 255), 1);

            // Figure out which sample zone had the lowest contrast from blue (lightest color)
            max = Math.max(avg1, Math.max(avg2, avg3));

            // Draw a circle on the detected skystone
            if(max == avg1) {
                skystone.x = (sub1PointA.x + sub1PointB.x) / 2;
                skystone.y = (sub1PointA.y + sub1PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 1;
            } else if(max == avg2) {
                skystone.x = (sub2PointA.x + sub2PointB.x) / 2;
                skystone.y = (sub2PointA.y + sub2PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 2;
            } else if(max == avg3) {
                skystone.x = (sub3PointA.x + sub3PointB.x) / 2;
                skystone.y = (sub3PointA.y + sub3PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 3;
            } else {
                position = 1;
            }

            // Free the allocated submat memory
            subMat1.release();
            subMat1 = null;
            subMat2.release();
            subMat2 = null;
            subMat3.release();
            subMat3 = null;

            return input;
        }
    }
}