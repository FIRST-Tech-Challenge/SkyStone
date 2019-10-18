package org.eastsideprep.eps15203;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Garage Servos Test [GarageTest] 15203", group="15203")

public class GarageTest15203 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware15203 robot = new Hardware15203();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "started");    //
        telemetry.update();

        waitForStart();

        for (int i=0; i<10; i++){
            robot.garageRightServo.setPower(-0.9);
            robot.garageLeftServo.setPower(-1.0);
            sleep(4000);
            robot.garageLeftServo.setPower(1.0);
            robot.garageRightServo.setPower(1.0);
            sleep(2000);
            robot.garageRightServo.setPower(0.0);
            robot.garageLeftServo.setPower(0.0);
            sleep(2000);
        }
    }
}

