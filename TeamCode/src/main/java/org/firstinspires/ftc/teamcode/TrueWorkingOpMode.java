package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TrueWorkingOpMode", group = "")
public class TrueWorkingOpMode extends LinearOpMode {

    private DcMotor motorDriveLeft;
    private DcMotor motorDriveRight;
    private Servo GripServo;

    @Override
    public void runOpMode () {
        motorDriveLeft = hardwareMap.dcMotor.get("motorDriveLeft");
        motorDriveRight = hardwareMap.dcMotor.get("motorDriveRight");
        GripServo = hardwareMap.servo.get("GripServo");
        waitForStart();
        if (opModeIsActive()) {
            motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            GripServo.setPosition(.5);
            while (opModeIsActive()) {
                if (opModeIsActive()) {
                    motorDriveLeft.setPower(-gamepad1.left_stick_y * .714285);
                    motorDriveRight.setPower(gamepad1.right_stick_y * 1);
                }
                if (gamepad1.left_bumper) {
                    GripServo.setPosition(.25);
                } else {
                    GripServo.setPosition(.5);
                }
                telemetry.update();
            }
            }
        }
    }

}