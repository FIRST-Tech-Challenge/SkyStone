package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class We_Know_This_Works extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    public DcMotor leverArm;

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive());
        robot.leverArm.setPower(gamepad1.left_stick_y);



            }

        }
