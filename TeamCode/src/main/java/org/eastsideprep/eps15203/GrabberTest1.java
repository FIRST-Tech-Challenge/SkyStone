package org.eastsideprep.eps15203;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name="Grabber Test 1", group="15203")

public class GrabberTest1 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware15203 robot = new Hardware15203();   // Use a Pushbot's hardware


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.grabberServo.setPosition(0);
        sleep(1000);
        robot.grabberServo.setPosition(0.1);
        sleep(1000);
        robot.grabberServo.setPosition(0.2);
        sleep(1000);
        robot.grabberServo.setPosition(0.3);
        sleep(1000);
        robot.grabberServo.setPosition(0.4);
        sleep(1000);
        robot.grabberServo.setPosition(0.5);
        sleep(1000);
        robot.grabberServo.setPosition(0.4);
        sleep(1000);
        robot.grabberServo.setPosition(0.3);
        sleep(1000);
        robot.grabberServo.setPosition(0.2);
        sleep(1000);
        robot.grabberServo.setPosition(0.1);
        sleep(1000);
        robot.grabberServo.setPosition(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "started");    //
        telemetry.update();
    }
}

