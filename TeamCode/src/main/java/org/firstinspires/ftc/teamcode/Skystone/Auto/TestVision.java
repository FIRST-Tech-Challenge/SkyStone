package org.firstinspires.ftc.teamcode.Skystone.Auto;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Skystone.Vision;

@TeleOp(name="TestVision", group = "Linear Opmode")
public class TestVision extends LinearOpMode {

    @Override
    public void runOpMode() {

        Vision tensorflow = new Vision(this);

        waitForStart();

        while (opModeIsActive()){
            Vision.Location position = tensorflow.runDetection2(false);
//            this.telemetry.addData("position: ", position);
//            this.telemetry.update();
//            sleep(3000);
//            this.telemetry.clear();
        }
    }
}
