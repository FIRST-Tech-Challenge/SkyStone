package org.eastsideprep.eps15203;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Autonomous WheelTest", group="15203")

public class Auto_Wheeltest15203 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware15203 robot = new Hardware15203();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "started");    //
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {
            robot.leftFrontMotor.setPower(1.0);
            sleep(1000);
            robot.leftFrontMotor.setPower(0.0);
            robot.leftBackMotor.setPower(1.0);
            sleep(
                    1000);
            robot.leftBackMotor.setPower(0.0);
            robot.rightFrontMotor.setPower(1.0);
            sleep(1000);
            robot.rightFrontMotor.setPower(0.0);
            robot.rightBackMotor.setPower(1.0);
            sleep(1000);
            robot.rightBackMotor.setPower(0.0);

            robot.leftFrontMotor.setPower(-1.0);
            sleep(1000);
            robot.leftFrontMotor.setPower(0.0);
            robot.leftBackMotor.setPower(-1.0);
            sleep(1000);
            robot.leftBackMotor.setPower(0.0);
            robot.rightFrontMotor.setPower(-1.0);
            sleep(1000);
            robot.rightFrontMotor.setPower(0.0);
            robot.rightBackMotor.setPower(-1.0);
            sleep(1000);
            robot.rightBackMotor.setPower(0.0);
        }

        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);

    }


}

