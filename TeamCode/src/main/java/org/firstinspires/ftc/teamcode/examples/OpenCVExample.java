package org.firstinspires.ftc.teamcode.examples;

import org.firstinspires.ftc.teamcode.opmodesupport.AutoOpMode;
import org.firstinspires.ftc.teamcode.roboticslibrary.FXTCamera;
import org.firstinspires.ftc.teamcode.util.OCVUtils;
import org.opencv.core.CvType;

/**
 * Created by Alec Krawciw on 2017-11-10.
 */

public class OpenCVExample extends AutoOpMode {
    FXTCamera cam;

    @Override
    public void runOp() throws InterruptedException {
        cam = new FXTCamera(FXTCamera.FACING_BACKWARD, true);//shows camera before init

        waitForStart();


        //Do some thing with opencv here example
        OCVUtils.bitmapToMat(cam.photo(), CvType.CV_8UC4);
    }

    public void stopOpMode(){
        cam.destroy();
    }
}
