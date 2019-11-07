package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.State;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;
import org.firstinspires.ftc.teamcode.listener.CameraListener;
import org.firstinspires.ftc.teamcode.listener.UltroVuforia;
import org.firstinspires.ftc.teamcode.monitor.MonitorCamera;
import org.firstinspires.ftc.teamcode.monitor.MonitorIMU;
import org.firstinspires.ftc.teamcode.monitor.MonitorManager;
import org.firstinspires.ftc.teamcode.opmode.AutoOpMode;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.Locale;

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
