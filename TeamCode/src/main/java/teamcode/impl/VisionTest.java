package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import teamcode.common.TTOpMode;
import teamcode.common.TTVision;

@Autonomous(name = "Vision Test")
public class VisionTest extends TTOpMode {

    private TTVision vision;

    @Override
    protected void onInitialize() {
        vision = new TTVision(hardwareMap);
        vision.enable();
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            List<Recognition> recognitions = vision.getRecognitions();
            for (Recognition recognition : recognitions) {
                telemetry.addData("Recognition", "x: %.2f y: %.2f", (recognition.getLeft() + recognition.getRight()) / 2, (recognition.getBottom() + recognition.getTop()) / 2);
            }
            telemetry.update();
        }
    }

    protected void onStop() {
        vision.disable();
    }

}
