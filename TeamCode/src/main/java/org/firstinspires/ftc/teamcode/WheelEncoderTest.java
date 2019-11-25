package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Drive Encoder", group="Exercises")
//@Disabled
public class WheelEncoderTest extends LinearOpMode
{
    private DcMotor front_left, front_right, back_left, back_right, intake_left, intake_right;

    @Override
    public void runOpMode() throws InterruptedException
    {
        front_left = hardwareMap.dcMotor.get("front_left"); // Port 0
        front_right = hardwareMap.dcMotor.get("front_right"); // Port 1
        back_left = hardwareMap.dcMotor.get("back_left"); // Port 2
        back_right = hardwareMap.dcMotor.get("back_right"); // Port 3

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        // reset encoder count kept by left motor.
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set left motor to run to 5000 encoder counts.

        drive_straight(5000, 0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && back_right.isBusy())
        {
            telemetry.addData("encoder-fwd", back_right.getCurrentPosition() + "  busy=" + back_right.isBusy());
            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        back_right.setPower(0.0);
        back_left.setPower(0.0);
        front_right.setPower(0.0);
        front_left.setPower(0.0);

        while (opModeIsActive() && getRuntime() < 5)
        {
            telemetry.addData("encoder-fwd-end", back_right.getCurrentPosition() + "  busy=" + back_right.isBusy());
            telemetry.update();
            idle();
        }

        // wait 5 sec so you can observe the final encoder position.

        stop();



        // wait 5 sec so you can observe the final encoder position.

    }
    public void drive_straight(int val, double pow){
        front_left.setTargetPosition(val);
        front_right.setTargetPosition(val);
        back_left.setTargetPosition(val);
        back_right.setTargetPosition(val);


        front_left.setPower(pow);
        front_right.setPower(pow);
        back_left.setPower(pow);
        back_right.setPower(pow);

    }
}