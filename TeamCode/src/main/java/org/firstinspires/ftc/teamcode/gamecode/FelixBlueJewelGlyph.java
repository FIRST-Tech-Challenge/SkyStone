//package org.firstinspires.ftc.teamcode.gamecode;
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
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.teamcode.R;
//import org.firstinspires.ftc.teamcode.RC;
//import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
//import org.firstinspires.ftc.teamcode.robots.Felix;
//import org.firstinspires.ftc.teamcode.util.OCVUtils;
//import org.opencv.core.CvType;
//import org.opencv.core.Mat;
//
//import static org.firstinspires.ftc.teamcode.gamecode.OpenCVLearning.getJewelConfig;
//
///**
// * Created by Aila on 2017-12-15.
// */
//@Disabled
//@Autonomous
//public class FelixBlueJewelGlyph extends AutoOpMode{
//    private Felix choc = null;
//
//    @Override
//    public void runOp() throws InterruptedException {
//
//        choc = new Felix();
//        choc.init(hardwareMap);
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
//            choc.holdGlyph();
//            choc.lift(1500, 0.8);
//
//            if (!getJewelConfig(newImg)) {
//                Log.i("Jewels", "Red Blue");
//                choc.leftJewel(true);
//                sleep(500);
//                choc.forwardDistance(25, 0.4);
//                choc.stop();
//                sleep(300);
//                choc.leftJewel(false);
//                sleep(200);
//                choc.forwardDistance(200, 0.4);
//                sleep(200);
//                choc.stop();
//                count = count + 1;
//            } else {
//                Log.i("Jewels", "Blue Red");
//                choc.leftJewel(true);
//                sleep(500);
//                choc.backwardDistance(25, 0.4);
//                choc.stop();
//                sleep(300);
//                choc.leftJewel(false);
//                sleep(200);
//                choc.forwardDistance(400, 0.4);
//                sleep(200);
//                choc.stop();
//                count = count + 1;
//            }
//
//            sleep(5000);
//
//        }
//
//    }
//}
