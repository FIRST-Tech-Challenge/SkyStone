package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "BasicTeleOp", group = "Basic")
public class BasicTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TypexChart chart = new TypexChart();

        chart.init(hardwareMap);
        waitForStart();

        //Setting drive parameters
        chart.TL.setPower(gamepad1.left_stick_y);
        chart.BL.setPower(gamepad1.left_stick_y);
        chart.TR.setPower(gamepad1.right_stick_y);
        chart.BR.setPower(gamepad1.right_stick_y);
        
    }
}

