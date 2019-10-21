package org.firstinspires.ftc.teamcode.examples;

import android.graphics.Bitmap;

import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.RC;
import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.OCVUtils;
import org.opencv.core.CvType;

/**
 * Created by Alec Krawciw on 2017-11-10.
 */

public class OpenCVVuforiaExample extends AutoOpMode {
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

        Bitmap b = OCVUtils.getVuforiaImage(locale.getFrameQueue().take(), PIXEL_FORMAT.RGB565);
        if(b != null){
            //Do openCV
            OCVUtils.bitmapToMat(b, CvType.CV_8UC3);
        }

    }
}
