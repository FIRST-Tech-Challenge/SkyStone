package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by maryjane on 11/6/2018.
 * after 1st meet
 * mecanum wheels
 */

@TeleOp(name="Holonomic", group="Iterative Opmode")
//@Disabled
public class HolonomicGyro extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declaration of the motors and servos goes here
    private DcMotor RL, RR, FL, FR;
    private Servo   servoLeft, servoRight;

    private double FLpower, FRpower, RLpower, RRpower;

    public static final double deadZone = 0.10;
    public static final boolean earthIsFlat = true;

    @Override //when init is pressed
    public void runOpMode(){

        //Naming, Initialization of the hardware, use this deviceName in the robot controller phone
        //use the name of the object in the code
        RL = hardwareMap.get(DcMotor.class, "left_drive");
        RR = hardwareMap.get(DcMotor.class, "right_drive");
        FL = hardwareMap.get(DcMotor.class, "front_left");
        FR = hardwareMap.get(DcMotor.class, "front_right");

        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        //servoClaw = hardwareMap.get(Servo.class, "servoClaw");

        //Set the direction of the motors
        //Reversed motors on one side to ensure forward movement.
        //invert all of them to change the robot's front/back
        RL.setDirection(DcMotorSimple.Direction.FORWARD);
        FL.setDirection(DcMotorSimple.Direction.FORWARD);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
        FR.setDirection(DcMotorSimple.Direction.REVERSE);

        //Running with/without Encoders
        RL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        runtime.reset();

        double speedSet = 5;//robot starts with speed 5 due to 40 ratio motors being op
        double reduction = 7.5;//fine rotation for precise stacking. higher value = slower rotation using triggers
        double maxPower = 0;

        while (opModeIsActive()) {

            //bumpers set speed of robot
            if(gamepad1.right_bumper)
                speedSet += 0.0005;
            else if(gamepad1.left_bumper)
                speedSet -= 0.0005;

            speedSet =  Range.clip(speedSet, 1, 10);//makes sure speed is limited at 10.

            if(!gamepad1.right_bumper && !gamepad1.left_bumper)//makes sure speed does not round every refresh. otherwise, speed won't be able to change
                speedSet = Math.round(speedSet);

            if(gamepad1.a) {
                servoLeft.setPosition(0.5);
                servoRight.setPosition(0.5);
            }
            else if(earthIsFlat) {
                servoLeft.setPosition(1);
                servoRight.setPosition(0);
            }

            //Holonomic Vector Math
            if((Math.abs(gamepad1.left_stick_x) > deadZone) || (Math.abs(gamepad1.left_stick_y) > deadZone) || (Math.abs(gamepad1.right_stick_x) > deadZone)) {
                FLpower = gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
                FRpower = gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
                RRpower = gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
                RLpower = gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            } else if(gamepad1.left_trigger > deadZone || gamepad1.left_trigger > deadZone){//we don't have to worry about Range.clip here because the abs values will never exceed 1
                FLpower = (-gamepad1.left_trigger + gamepad1.right_trigger)/reduction;
                FRpower = (gamepad1.left_trigger - gamepad1.right_trigger)/reduction;
                RRpower = (gamepad1.left_trigger - gamepad1.right_trigger)/reduction;
                RLpower = (-gamepad1.left_trigger + gamepad1.right_trigger)/reduction;
            } else if (earthIsFlat) {//stop robot
                FLpower = 0;
                FRpower = 0;
                RLpower = 0;
                RRpower = 0;
            }

            //get max power out of all 4 powers
            maxPower = Math.max(1.0, Math.max(Math.max(Math.abs(FLpower), Math.abs(RLpower)), Math.max(Math.abs(FRpower), Math.abs(RRpower))));

            //if any of them is greater than 1, it will slow down all by the same ratio
            if(maxPower > 1.0) {
                FLpower /= maxPower;
                FRpower /= maxPower;
                RLpower /= maxPower;
                RRpower /= maxPower;
            }

            FL.setPower(FLpower*speedSet/10);
            FR.setPower(FRpower*speedSet/10);
            RL.setPower(RLpower*speedSet/10);
            RR.setPower(RRpower*speedSet/10);

            telemetry.addData("Drive", "Holonomic");
            telemetry.addData("Left", servoLeft.getPosition());
            telemetry.addData("Right", servoRight.getPosition());
            telemetry.addData("speedSet", "%.2f", speedSet);
            telemetry.update();
        }
    }
}
