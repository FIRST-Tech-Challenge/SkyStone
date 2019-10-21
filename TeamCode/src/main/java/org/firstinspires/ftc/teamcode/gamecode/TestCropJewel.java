package org.firstinspires.ftc.teamcode.gamecode;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.robots.Felix;
import org.firstinspires.ftc.teamcode.util.OCVUtils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.teamcode.gamecode.OpenCVLearning.getJewelConfig;

/**
 * Created by Aila on 2018-01-19.
 */

@Autonomous
@Disabled
public class TestCropJewel extends AutoOpMode {

    private Felix gem = null;

    VuforiaLocalizer vuforia;

    OpenGLMatrix lastLocation = null;


    @Override
    public void runOp() throws InterruptedException {

        gem = new Felix();
        gem.init(hardwareMap);

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

            Mat rimg = OCVUtils.rotate(img, 180);
            Mat newImg = rimg.submat(309, 610, 578, 1222);

            OCVUtils.saveToInternalStorage(newImg, "FieldPic");

            if (!getJewelConfig(newImg)) {
                telemetry.addData("Jewels", "Red Blue");
                gem.wheelL.setPower(0.5);
            } else {
                telemetry.addData("Jewels", "Blue Red");
                gem.wheelR.setPower(0.5);
            }

        }

    }
}
