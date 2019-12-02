package org.firstinspires.ftc.teamcode.Skystone.Auto;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Skystone.Vision;

@TeleOp(name="TestVision", group = "Linear Opmode")
public class TestVision extends LinearOpMode {

    @Override
    public void runOpMode() {

        Vision tensorflow = new Vision(this);

        waitForStart();

            try {
                Vision.Location position = tensorflow.runDetection3(false);
                telemetry.addLine("Detection: " + position.toString());
                telemetry.update();
                Log.e("Vision", position.toString());
            }catch (Exception e){
                Log.e("Vision", "Error", e);
            }
//            this.telemetry.addData("position: ", position);
//            this.telemetry.update();
            sleep(3000);
//            this.telemetry.clear();

    }
}
