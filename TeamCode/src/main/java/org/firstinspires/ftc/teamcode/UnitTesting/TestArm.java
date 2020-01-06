package org.firstinspires.ftc.teamcode.UnitTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TestArm", group = "TeleopTest")
public class TestArm extends LinearOpMode {

    DcMotor armTest;
    @Override

    public void runOpMode() throws InterruptedException {

        armTest = hardwareMap.dcMotor.get("armMotor");
        armTest.setPower(1);


        }
}
