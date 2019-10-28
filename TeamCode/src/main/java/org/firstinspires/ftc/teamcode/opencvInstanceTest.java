package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import pkg3939.skystoneDetectorClass;

/**
 * Created by maryjaneb  on 11/13/2016.
 *
 * nerverest ticks
 * 60 1680
 * 40 1120
 * 20 560
 *
 * monitor: 640 x 480
 */
@Autonomous(name= "opencvInstanceTest", group="Sky autonomous")
//@Disabled
public class opencvInstanceTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    skystoneDetectorClass detector = new skystoneDetectorClass(
    );

    int[] vals;

    @Override
    public void runOpMode() throws InterruptedException {
        detector.setOffset(-1f/8f, 3f/8f);
        detector.camSetup(hardwareMap);

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            detector.updateVals();
            vals = detector.getVals();
            telemetry.addData("Values", vals[1]+"   "+vals[0]+"   "+vals[2]);
            telemetry.update();
            sleep(100);

        }
    }


}