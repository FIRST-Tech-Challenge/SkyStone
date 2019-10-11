package teamcode.impl;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import teamcode.common.TTOpMode;
import teamcode.common.TTVision;
import teamcode.common.Vector2;

@Autonomous(name = "Vision Bounding Box Finder")
public class VisionBoundingBoxFinder extends TTOpMode {

    private TTVision vision;

    @Override
    protected void onInitialize() {
        vision = new TTVision(hardwareMap);
        vision.enable();
        telemetry.addData("description", "This op mode displays the coordinates of bounding boxes that encompass recognitions");
        telemetry.update();
    }

    @Override
    protected void onStart() {
        while (opModeIsActive()) {
            List<Recognition> recognitions = vision.getRecognitions();
            for (Recognition recognition : recognitions) {
                Vector2 topLeft = TTVision.getTopLeft(recognition).invert();
                Vector2 bottomRight = TTVision.getBottomRight(recognition).invert();
                String label = recognition.getLabel();
                telemetry.addData(label, "{%s, %s}", topLeft, bottomRight);
            }
            telemetry.update();
        }
    }

    @Override
    protected void onStop() {
        vision.disable();
    }

}
