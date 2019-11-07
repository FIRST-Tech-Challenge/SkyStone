package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;

import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name="Camera: help me", group="Linear Opmode")
public class CameraTestAuto extends AutoOpMode {
    @Override
    public void setup(DeviceMap map) {
        //map.setUpVuforia(hardwareMap);
        map.initOpenCV(hardwareMap);
        map.initTfod(hardwareMap);

        map.getCamera().setPipeline(new Pipeline());
        telemetry.update();
        DeviceMap.getInstance().getCamera().startStreaming(320, 240);
    }

    @Override
    public void beforeLoop() {


    }

    @Override
    public void run() {
    }

    private class Pipeline extends OpenCvPipeline {

        public Mat processFrame(Mat mat) {
            telemetry.addLine("updating image");
            telemetry.update();
            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2GRAY);
            return mat;
        }
    }


}
