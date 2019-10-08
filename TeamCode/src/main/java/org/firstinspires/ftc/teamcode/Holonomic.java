package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by maryjane on 11/6/2018.
 * after 1st meet
 * mecanum wheels
 */

@TeleOp(name="Holonomic", group="Iterative Opmode")
//@Disabled
public class Holonomic extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declaration of the motors and servos goes here
    private DcMotor backLeft     = null; //rear left
    private DcMotor backRight    = null; //rear right
    private DcMotor frontLeft    = null; //front left
    private DcMotor frontRight   = null; //front right

    public static final double deadZone = 0.10;
    public static final boolean earthIsFlat = true;

    @Override //when init is pressed
    public void runOpMode(){

        //Naming, Initialization of the hardware, use this deviceName in the robot controller phone
        //use the name of the object in the code
        backLeft = hardwareMap.get(DcMotor.class, "left_drive");
        backRight = hardwareMap.get(DcMotor.class, "right_drive");
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");

        //Set the direction of the motors
        //Reversed motors on one side to ensure forward movement.
        //invert all of them to change the robot's front/back
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Zero Power Behavior -> use only for autonomous for precious movement

        //backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Running with/without Encoders
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        runtime.reset();
        double speedSet = 5;//robot starts with 5 speed due to 40 ratio motors being op

        while (opModeIsActive()) {

            //bumpers set speed of robot
            if(gamepad1.right_bumper)
                speedSet += 0.001;
            else if(gamepad1.left_bumper)
                speedSet -= 0.001;

            speedSet =  Range.clip(speedSet, 1, 10);//makes sure speed is limited at 10.

            if(!gamepad1.right_bumper && !gamepad1.left_bumper)//makes sure speed does not round every refresh. otherwise, speed is "pulled back" by the round
                speedSet = Math.round(speedSet);

            //directional
            //using range.clip makes sure you can use all sticks and directions at the same time without conflicts. power stays limited at 1
            if((Math.abs(gamepad1.left_stick_x) > deadZone) || (Math.abs(gamepad1.left_stick_y) > deadZone) || (Math.abs(gamepad1.right_stick_x) > deadZone)) {
                frontLeft.setPower(Range.clip((gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x) * (speedSet / 10), 0, 1));
                frontRight.setPower(Range.clip((gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x) * (speedSet / 10), 0, 1));
                backRight.setPower(Range.clip((gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) * (speedSet / 10), 0, 1));
                backLeft.setPower(Range.clip((gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x) * (speedSet / 10), 0, 1));
            } else if (earthIsFlat) {//stop robot
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
                backLeft.setPower(0);
            }

            telemetry.addData("Drive", "Holonomic");
            telemetry.addData("speedSet", "%.2f", speedSet);
            telemetry.update();

        }


    }
}
