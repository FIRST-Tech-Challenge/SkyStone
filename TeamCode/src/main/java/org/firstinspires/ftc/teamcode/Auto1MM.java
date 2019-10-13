package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// @Disabled
@Autonomous (name = "MMAuto1")
public class Auto1MM extends LinearOpMode {

    // drive motor declaration
    DcMotor left;
    DcMotor right;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive Motor instantiation
        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        // Drive Motor Direction
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        left.setPower(0.0);
        right.setPower(0.0);

        waitForStart();

        left.setPower(1.0);
        right.setPower(1.0);

        sleep(5000);

        left.setPower(0.0);
        right.setPower(0.0);
    }
}
