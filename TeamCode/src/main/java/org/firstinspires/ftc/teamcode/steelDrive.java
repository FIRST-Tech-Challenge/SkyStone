package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "steelDrive", group = "steel")
public class steelDrive extends LinearOpMode {
    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    @Override
    public void runOpMode() throws InterruptedException {
        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        while (opModeIsActive()) {
            TL.setPower(gamepad1.left_stick_y);
            BL.setPower(gamepad1.left_stick_y);
            TR.setPower(gamepad1.right_stick_y);
            BR.setPower(gamepad1.right_stick_y);

        }
    }
}
