package org.eastsideprep.eps15203;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="SpinTurn", group="15203")

public class SpinTurn extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware15203 robot = new Hardware15203();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "started");    //
        telemetry.update();


        waitForStart();

        while (opModeIsActive()){
        //Front Motors
            leftFrontMotor.setPower(-0.5);
            rightFrontMotor.setPower(0.5);
        //Back motors
            leftBackMotor.setPower(0.5);
            rightBackMotor.setPower(0.5);
        }
/*
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
        }
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        */
    }
}
