package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "BasicTeleOp", group = "Basic")
public class BasicTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TypexChart chart = new TypexChart();

        chart.init(hardwareMap);
        waitForStart();

        //Setting drive parameters
        while(opModeIsActive()) {

            //Strafe Code and Standard Drive Code
            if (gamepad1.dpad_left) {
                chart.TL.setPower(1);
                chart.BL.setPower(-1);
                chart.TR.setPower(-1);
                chart.BR.setPower(1);
            }
            else if (gamepad1.dpad_right) {
                chart.TL.setPower(-1);
                chart.BL.setPower(1);
                chart.TR.setPower(1);
                chart.BR.setPower(-1);
            }
            else {
                chart.TL.setPower(gamepad1.left_stick_y);
                chart.BL.setPower(gamepad1.left_stick_y);
                chart.TR.setPower(gamepad1.right_stick_y);
                chart.BR.setPower(gamepad1.right_stick_y);
            }

        }
    }
}

