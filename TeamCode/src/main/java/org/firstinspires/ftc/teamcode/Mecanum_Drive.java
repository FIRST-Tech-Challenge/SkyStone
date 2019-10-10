package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class Mecanum_Drive extends LinearOpMode {
    private DcMotor frontleft, frontright, backleft, backright;
    float fwd, side, turn, power;
    public static double deadzone = 0.2;

    @Override
    public void runOpMode() {
        frontleft = hardwareMap.get(DcMotor.class, "front_left");
        frontright = hardwareMap.get(DcMotor.class, "front_right");
        backleft = hardwareMap.get(DcMotor.class, "back_left");
        backright = hardwareMap.get(DcMotor.class, "back_right");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            getGamePadStickValues();

            power = fwd; //this can be tweaked for exponential power increase

            frontleft.setPower(Range.clip(power + side - turn, -1, 1));
            frontright.setPower(Range.clip(power - side + turn, -1, 1));
            backleft.setPower(Range.clip(power - side - turn, -1, 1));
            backright.setPower(Range.clip(power + side + turn, -1, 1));

            telemetry.addData("Motor Power FL ", frontleft.getPower());
            telemetry.addData("Motor Power FR", frontright.getPower());
            telemetry.addData("Motor Power BL", backleft.getPower());
            telemetry.addData("Motor Power BR", backright.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }

    void getGamePadStickValues() {
        fwd = gamepad1.left_stick_y;
        side = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;
        //updates joystick values

        if( Math.abs(fwd) < deadzone ) fwd = 0;
        if( Math.abs(side) < deadzone ) side = 0;
        if( Math.abs(turn) < deadzone ) turn = 0;
        //checks deadzones
    }
}
