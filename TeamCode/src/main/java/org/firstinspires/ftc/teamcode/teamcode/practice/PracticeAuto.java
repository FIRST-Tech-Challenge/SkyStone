package org.firstinspires.ftc.teamcode.teamcode.practice;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name= "Practice Autonomous", group= "PracticeAuto")
public class PracticeAuto extends LinearOpMode {


    DcMotor leftWheel;
    DcMotor rightWheel;

    @Override
    public void runOpMode() throws InterruptedException {

        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");

        leftWheel.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        leftWheel.setPower(1);
        rightWheel.setPower(1);

        sleep(1000);

        leftWheel.setPower(0);
        rightWheel.setPower(0);

        sleep(1000);

        leftWheel.setPower(-.5);
        rightWheel.setPower(.5);

        sleep(1000);

        leftWheel.setPower(0);
        rightWheel.setPower(0);




    }
}
