package org.firstinspires.ftc.teamcode.Skystone.Auto;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Skystone.Vision;

@Disabled
@TeleOp(name="TestVision", group = "Linear Opmode")
public class TestVision extends LinearOpMode {

    @Override
    public void runOpMode() {

        Vision tensorflow = new Vision(this);

        waitForStart();

            try {
                tensorflow.captureFrameToFile();
            }catch (Exception e){
                Log.e("Vision", "Error", e);
            }
//            this.telemetry.addData("position: ", position);
//            this.telemetry.update();
            sleep(3000);
//            this.telemetry.clear();

    }
}
