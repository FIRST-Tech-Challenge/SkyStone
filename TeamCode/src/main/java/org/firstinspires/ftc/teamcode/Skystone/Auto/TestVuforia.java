package org.firstinspires.ftc.teamcode.Skystone.Auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="TestVuforia")
public class TestVuforia extends AutoBase {

    @Override
    public void runOpMode() {
        initLogic();

        waitForStart();
        int tfodPosition = robot.detectTensorflow();
        robot.goToSkystone(tfodPosition);
    }
}
