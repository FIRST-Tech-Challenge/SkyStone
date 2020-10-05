package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Supernova11567Main", group = "Iterative Opmode")

public class Supernova11567Main extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void init() {
        telemetry.addData("Status", "INIT");
        runtime.reset();
    }


    @Override
    public void init_loop() {
    }


    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

    }


    @Override
    public void stop() {
    }

}
