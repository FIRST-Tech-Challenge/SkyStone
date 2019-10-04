package org.firstinspires.ftc.teamcode.sensors;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareBeep;

@Autonomous(name = "Testing Ultrasonic Sensor", group = "Exercises")
public class TestingMB1242Sensor extends LinearOpMode {

    public HardwareBeep robot = new HardwareBeep();
    boolean readLeftSensor = false;

//    Parameters

//    address: the 7-bit I2C device address of the device to transmit to and from.
//    quantity: the number of bytes to request.
//    value: a value to send as a single byte.

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime runtime = new ElapsedTime();
        robot.init(hardwareMap);
        int i = 0;

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        waitForStart();

        runtime.reset();

        while (true) {

            if (!readLeftSensor && runtime.milliseconds() > 100) {
                telemetry.addData("Left Distance", robot.leftSonic.getDistance());
                telemetry.addData("Incrementor", i++);
                telemetry.update();
                robot.leftSonic.ping();
                readLeftSensor = true;

            }

            if (runtime.milliseconds() > 200) {

                telemetry.addData("Right Distance", robot.rightSonic.getDistance());
                telemetry.addData("Incrementor", i++);
                telemetry.update();
                robot.rightSonic.ping();
                readLeftSensor = false;
                runtime.reset();
            }
        }
    }
}