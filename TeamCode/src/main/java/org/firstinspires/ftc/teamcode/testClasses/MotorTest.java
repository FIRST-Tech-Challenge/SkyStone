package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorTest extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() {


        telemetry.setMsTransmissionInterval(1);
        telemetry.addLine("Init | v1.0");
        waitForStart();
        while (opModeIsActive()) {
            //Other unit test code if you want
            telemetry.update();
        }
    }
}