package org.firstinspires.ftc.teamcode.Skystone.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Skystone.Vision;

@Autonomous(name="TestVision")
public class TestVision extends AutoBase {

    @Override
    public void runOpMode() {

        initLogic();
        Vision tensorflow = new Vision(robot);

//        tensorflow.initVision();
        waitForStart();

        //robot.moveToPoint(8.5,0,0.5,1,Math.toRadians(0));

        Vision.Location position = tensorflow.runDetection();
        robot.getTelemetry().addLine("position: " + position);
        robot.getTelemetry().update();
        sleep(10000);

    }
}
