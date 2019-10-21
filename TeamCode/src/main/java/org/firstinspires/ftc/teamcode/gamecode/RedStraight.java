//package org.firstinspires.ftc.teamcode.gamecode;
//
//import android.graphics.Bitmap;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.vuforia.PIXEL_FORMAT;
//import com.vuforia.Vuforia;
//
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
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
// * Created by Aila on 2018-01-22.
// */
//
//@Autonomous
//@Disabled
//public class RedStraight extends AutoOpMode {
//
//    private Felix bot = null;
//
//    VuforiaLocalizer vuforia;
//
//    OpenGLMatrix lastLocation = null;
//
//
//    @Override
//    public void runOp() throws InterruptedException {
//
//        bot = new Felix();
//        bot.init(hardwareMap);
//
//        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        params.vuforiaLicenseKey = RC.VUFORIA_LICENSE_KEY;
//        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//        VuforiaLocalizer locale = ClassFactory.createVuforiaLocalizer(params);
//        locale.setFrameQueueCapacity(1);
//        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//
//
//        waitForStart();
//
//
//
//        Bitmap jewelImage = OCVUtils.getVuforiaImage(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
//        Mat img = OCVUtils.bitmapToMat(jewelImage, CvType.CV_8UC4);
//
//        Mat rimg = OCVUtils.rotate(img, 180);
//        Mat newImg = rimg.submat(309, 610, 578, 1222);
//
//        OCVUtils.saveToInternalStorage(newImg, "FieldPic");
//
//        bot.jewelR.setPosition(0);
//        sleep(1500);
//
//        if (!getJewelConfig(newImg)) {
//            telemetry.addData("Jewels", "Red Blue");
//            bot.forwardDistance(18, 0.2);
//            bot.stop();
//        } else {
//            telemetry.addData("Jewels", "Blue Red");
//            bot.backward(18, 800);
//            bot.stop();
//        }
//
//        sleep(1500);
//        bot.jewelR.setPosition(1);
//        sleep(1500);
//        bot.stop();
//
//    }
//
//}
