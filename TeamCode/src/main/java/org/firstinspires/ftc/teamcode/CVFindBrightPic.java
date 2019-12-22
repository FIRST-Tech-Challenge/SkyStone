package org.firstinspires.ftc.teamcode;

import java.io.File;
import java.util.ArrayList;
import java.util.Vector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * 2019.10.19
 * Athena Z.
 */

@TeleOp(name = "CVFindBrightPic", group = "Test")
public class CVFindBrightPic extends LinearOpMode {

    RobotProfile robotProfile;

    private int skystonePosition = -1;
    private boolean snappedPic = false;
    private boolean pressedA = false;

    RobotProfile.StartPosition startPosition = RobotProfile.StartPosition.RED_2;
    ArrayList<RobotControl> taskList = new ArrayList<>();

    @Override
    public void runOpMode() {
        try{
            robotProfile = RobotProfile.loadFromFile(new File("/sdcard/FIRST/profile.json"));
        } catch (Exception e) {
        }
        taskList.add(new RobotSleep(3000));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        OpenCvCamera phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new Pipeline());
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        RobotControl task = taskList.get(0);
        task.prepare();

        while (opModeIsActive()) {
            task.execute();

            telemetry.addData("Task is done", task.isDone());

            telemetry.addData("Skystone Position", skystonePosition);

            telemetry.update();

            if (gamepad1.a) {
                if (!pressedA) {
                    snappedPic = true;
                }
                pressedA = true;
            }
            else {
                pressedA = false;
            }
        }
    }

    public class Pipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {

            Scalar rectColor = new Scalar(0, 0, 255); //red color
            //TODO: the 3 stones' center coordinates are hardcoded for now, but will be not later on

            //gets point array
            RobotProfile.Point [] points = robotProfile.stoneScanPoints.get(startPosition);
            int rectLength = 30;
            int rectHalf = rectLength / 2;
            for(int i = 0; i < 3; i++){
                Imgproc.rectangle(input, new Rect(points[i].x - rectHalf, points[i].y - rectHalf, 30, 30), rectColor);
            }

            Mat imgBright = new Mat();

            //convert normal picture into HSV picture (deals with brightness)
            Imgproc.cvtColor(input, imgBright, Imgproc.COLOR_BGR2HSV);

            //get the V channel image of imgBright and save as a new image
            Vector<Mat> hsv_planes = new Vector<>();
            Core.split(imgBright, hsv_planes);

            Mat imgV = hsv_planes.get(2);

            double darkestVal = 500;

            //crop imgV based on the 3 rectangles and save each as a new image in rectCrops
            for(int i = 0; i < 3; i++){
                Imgproc.rectangle(input, new Rect(points[i].x - rectHalf, points[i].y - rectHalf, 30, 30), rectColor);
                Mat subPic = new Mat(imgV,  new Rect(points[i].x - rectHalf, points[i].y - rectHalf, 30, 30));
                Scalar temp = Core.mean(subPic);
                if(temp.val[0] < darkestVal){
                    darkestVal = temp.val[0];
                    skystonePosition = i;
                }
            }
            if (snappedPic) {

                //need to save pic to file
                Mat mbgr = new Mat();
                Imgproc.cvtColor(input, mbgr, Imgproc.COLOR_RGB2BGR, 3);
                Imgcodecs.imwrite("/sdcard/FIRST/S" + System.currentTimeMillis()%1000 + ".jpg", mbgr);
                mbgr.release();

                snappedPic = false;
            }

            return input;
        }
    }
}