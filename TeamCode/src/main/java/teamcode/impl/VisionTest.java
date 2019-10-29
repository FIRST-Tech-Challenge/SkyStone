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
        //vision = getRobot().getVision();
        vision.enable();
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            List<Recognition> recognitions = vision.getRecognitions();
            for (Recognition recognition : recognitions) {
                telemetry.addData("Recognitions", recognition.getLabel());
            }
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {

    }

}
