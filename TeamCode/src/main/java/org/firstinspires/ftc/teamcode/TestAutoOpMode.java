// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Test", group="Exercises")

public class TestAutoOpMode extends LinearOpMode {

    FourWheelsDriveBot robot = new FourWheelsDriveBot(this);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.testOneMotor( robot.leftFront, 0.3, 1);
        robot.testOneMotor( robot.rightFront, 0.3, 1);
        robot.testOneMotor( robot.leftRear, 0.3, 1);
        robot.testOneMotor( robot.rightRear,0.3, 1);
    }

}
