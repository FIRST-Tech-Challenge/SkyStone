package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class FreshStartMode extends OpMode {

    private DcMotor lf, lb, rf, rb;

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("front_left");
    }

    @Override
    public void init_loop() {
        telemetry.addLine(Integer.toString(lf.getPortNumber()));
    }

    @Override
    public void loop() {
        telemetry.addLine("ur mum lol");
    }
}
