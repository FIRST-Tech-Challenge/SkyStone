package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Camera Test", group="Tests")

public class CameraTest extends LinearOpMode {
    private CameraBot robot = new CameraBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {

            int skystonePosition = robot.detectSkystone();

            telemetry.log().add("I think the Skystone is in Position %d", skystonePosition);
            telemetry.update();
            sleep (100 * 1000);
            break;
        }
    }
}