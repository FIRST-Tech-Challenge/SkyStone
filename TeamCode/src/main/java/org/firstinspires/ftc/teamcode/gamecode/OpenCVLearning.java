package org.firstinspires.ftc.teamcode.gamecode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.OCVUtils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

/**
 * Created by Windows on 2017-11-10.
 */
public class OpenCVLearning extends AutoOpMode{

    // static final boolean ON_LEFT = true;


    public static boolean getJewelConfig (Mat img){

        Mat pic = new Mat();
        Mat pic2 = new Mat();
        Mat pic3 = new Mat();

        // convert to hsv
        Imgproc.cvtColor(img,img,Imgproc.COLOR_RGB2HSV);
        Core.inRange(img, new Scalar (100,50,50), new Scalar (110,255,255), pic2);
        Core.inRange(img, new Scalar (160,50,50), new Scalar (180,255,255), pic);
        Core.inRange(img, new Scalar (0, 50, 50), new Scalar(10, 255, 255), pic3);
        Core.add(pic, pic3, pic); //adds pic and pic 3 and puts it in pic



        Moments centreRed = Imgproc.moments(pic, true);
        int centreXRed = (int) (centreRed.get_m10() / centreRed.get_m00());
        int centreYRed = (int) (centreRed.get_m01() / centreRed.get_m00());

        Moments centreBlue = Imgproc.moments(pic2, true);
        int centreXBlue = (int) (centreBlue.get_m10()/ centreBlue.get_m00());
        int centreYBlue = (int) (centreBlue.get_m01()/ centreBlue.get_m00());



        // saves binary images
        Imgproc.cvtColor(pic, pic, Imgproc.COLOR_GRAY2BGR);
        Imgproc.cvtColor(pic2, pic2, Imgproc.COLOR_GRAY2BGR);
        OCVUtils.saveToInternalStorage(pic, "RedJewelBinary");
        OCVUtils.saveToInternalStorage(pic2, "BlueJewelBinary");

        return (centreXBlue < centreXRed);
    } // getJewelConfig

    @Override
    public void runOp() throws InterruptedException {
        //do the camera thing before init to see the camera before init
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
        locale.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        waitForStart();

        while (opModeIsActive()) {
            Bitmap jewelImage = OCVUtils.getVuforiaImage(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
            Mat img = OCVUtils.bitmapToMat(jewelImage, CvType.CV_8UC4);
            // Mat newImg = img.submat(989, 1477, 624, 2854);
            if (getJewelConfig(img)) {
                System.out.println("The blue jewel is on the left side.");
                telemetry.addData("Jewels", "Blue Red");
            } else {
                System.out.println("The blue jewel is on the right side.");
                telemetry.addData("Jewels", "Red Blue");
            }
        }
    }
}
