package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;

import org.firstinspires.ftc.robotcontroller.ultro.listener.CameraListener;
import org.firstinspires.ftc.robotcontroller.ultro.listener.UltroVuforia;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

@Autonomous(name="Camera: help me", group="Linear Opmode")
public class CameraTestAuto extends AutoOpMode implements CameraListener {
    @Override
    public void setup(DeviceMap map) {
        map.setUpVuforia(hardwareMap);
        map.initTfod(hardwareMap);

        //MonitorManager.startAll(map);
        UltroVuforia.addListener(this);
        telemetry.update();
    }

    @Override
    public void beforeLoop() {


    }

    @Override
    public void run() {

    }

    public void process(Image image, Mat mat) {
        telemetry.addLine("updating image");
        telemetry.update();
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGBA2GRAY);
    }


}
