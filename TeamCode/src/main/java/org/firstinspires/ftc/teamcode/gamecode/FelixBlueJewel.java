//package org.firstinspires.ftc.teamcode.gamecode;
//
//import java.util.Calendar;
//
//import android.graphics.Bitmap;
//import android.util.Log;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.vuforia.PIXEL_FORMAT;
//import com.vuforia.Vuforia;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.teamcode.R;
//import org.firstinspires.ftc.teamcode.RC;
//import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
//import org.firstinspires.ftc.teamcode.robots.Felix;
//import org.firstinspires.ftc.teamcode.util.OCVUtils;
//import org.opencv.core.Core;
//import org.opencv.core.CvType;
//import org.opencv.core.Mat;
//import org.opencv.imgproc.Imgproc;
//
//import static org.firstinspires.ftc.teamcode.gamecode.OpenCVLearning.getJewelConfig;
//
///**
// * Created by Aila on 2017-12-10.
// */
//@Disabled
//@Autonomous
//public class FelixBlueJewel extends AutoOpMode{
//
//    private Felix chai = null;
//
//    @Override
//    public void runOp() throws InterruptedException {
//
//        chai = new Felix();
//        chai.init(hardwareMap);
//
//        chai.releaseGlyph();
//
//        int count = 0;
//
//        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
//        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
//        locale.setFrameQueueCapacity(1);
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//
//        waitForStart();
//
//        while (opModeIsActive() && count < 1) {
//            Bitmap jewelImage = OCVUtils.getVuforiaImage(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
//            Mat img = OCVUtils.bitmapToMat(jewelImage, CvType.CV_8UC4);
//
//            Mat rimg = OCVUtils.rotate(img, 180);
//            Mat newImg = rimg.submat(309, 610, 578, 1222);
//
//            OCVUtils.saveToInternalStorage(newImg, "FieldPic");
//
//            chai.holdGlyph();
//            chai.lift(1500, 0.8);
//
//            if (!getJewelConfig(newImg)) {
//                Log.i("Jewels", "Red Blue");
//                chai.leftJewel(true);
//                sleep(500);
//                chai.forwardDistance(35, 0.4);
//                chai.stop();
//                sleep(300);
//                chai.leftJewel(false);
//                sleep(200);
//                chai.backwardDistance(400, 0.4);
//                sleep(200);
//                chai.stop();
//                count = count + 1;
//            } else {
//                Log.i("Jewels", "Blue Red");
//                chai.leftJewel(true);
//                sleep(500);
//                chai.backwardDistance(35, 0.4);
//                chai.stop();
//                sleep(300);
//                chai.leftJewel(false);
//                sleep(200);
//                chai.backwardDistance(400, 0.4);
//                sleep(200);
//                chai.stop();
//                count = count + 1;
//            }
//
//            sleep(5000);
//
//        }
//
//    }
//}
