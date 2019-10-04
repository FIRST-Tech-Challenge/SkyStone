package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

/**
 * Created by maryjane on 11/6/2018.
 * after 1st meet
 * mecanum wheels
 */

@TeleOp(name="TeleOpMecanum", group="Iterative Opmode")
//@Disabled
public class TeleOpMecanum extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //Declaration of the motors and servos goes here
    private DcMotor backLeft     = null; //rear left
    private DcMotor backRight    = null; //rear right
    private DcMotor frontLeft    = null; //front left
    private DcMotor frontRight   = null; //front right
    //private DcMotor lift         = null; //lift motor
    //private DcMotor armMotor     = null;
    //private Servo   claw         = null; //the servo on the claw

    //positioning of the servos
    public static final double OPEN_SERVO  = 0.10; //sets the positions of the servo to max open
    public static final double CLOSE_SERVO = 0.50; //sets the position of the servo to max close

    @Override //when init is pressed
    public void init(){
        //debugging using the phone
        telemetry.addData("status","Initialized");

        //Naming, Initialization of the hardware
        backLeft       = hardwareMap.dcMotor.get("left_drive");
        backRight      = hardwareMap.dcMotor.get("right_drive");
        frontLeft       = hardwareMap.dcMotor.get("front_left");
        frontRight      = hardwareMap.dcMotor.get("front_right");
        //lift            = hardwareMap.dcMotor.get("lift");
        //armMotor        = hardwareMap.dcMotor.get("arm_motor");
        //claw            = hardwareMap.servo.get("claw");

        //Set the direction of the motors
        //Reversed motors
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
      // lift.setDirection(DcMotorSimple.Direction.FORWARD);
       // armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //Initial positioning of the servos
      //  claw.setPosition(CLOSE_SERVO);

        //Running with or without Encoders
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override //loop after init code runs, until start is pressed
    public void init_loop() {
    }

    @Override //run once after pressing start
    public void start(){
        runtime.reset();
    }

    @Override //run loop after start code finishes
    public void loop(){
        double forwardPower  = 0.0;
        double strafeRight = 0.0;
        double strafeLeft = 0.0;
        double turnPower = 0.0;
      //  boolean liftUp   = false;
       // boolean liftDown = false;
     //   boolean armUp = false;
      //  boolean armDown = false;
      //  boolean clawOpen = false;
      //  boolean clawClose = false;
      //  double clawPosition = CLOSE_SERVO;

        //gamepad1
        forwardPower   = gamepad1.left_stick_y;
        strafeRight = gamepad1.right_trigger;
        strafeLeft  = gamepad1.left_trigger;
        turnPower = gamepad1.right_stick_x;
      //  liftDown = gamepad1.dpad_down;
      //  liftUp = gamepad1.dpad_up;
      //  armUp = gamepad1.y;
      //  armDown = gamepad1.x;
       // clawOpen    = gamepad1.a;
      //  clawClose   = gamepad1.b;

        //move forward/backward
        if(abs(forwardPower) > 0.05){
            backLeft.setPower(forwardPower);
            backRight.setPower(forwardPower);
            frontLeft.setPower(forwardPower);
            frontRight.setPower(forwardPower);
        } else {
            forwardPower = 0;
            backLeft.setPower(forwardPower);
            backRight.setPower(forwardPower);
            frontLeft.setPower(forwardPower);
            frontRight.setPower(forwardPower);
        }

        //strafe left/Right
        if(abs(strafeRight) > 0.05) {
            backLeft.setPower(strafeRight);
            backRight.setPower(-strafeRight);
            frontLeft.setPower(-strafeRight);
            frontRight.setPower(strafeRight);
        } else {
            strafeRight = 0;
            backLeft.setPower(strafeRight);
            backRight.setPower(strafeRight);
            frontLeft.setPower(strafeRight);
            frontRight.setPower(strafeRight);
        }

        if(abs(strafeRight) > 0.05) {
            backLeft.setPower(strafeRight);
            backRight.setPower(-strafeRight);
            frontLeft.setPower(-strafeRight);
            frontRight.setPower(strafeRight);
        } else if(abs(strafeLeft) > 0.05) {
            backLeft.setPower(-strafeLeft);
            backRight.setPower(strafeLeft);
            frontLeft.setPower(strafeLeft);
            frontRight.setPower(-strafeLeft);
        } else {
            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }

        //turn
        if(abs(turnPower) > 0.05) {
            backRight.setPower(turnPower);
            frontRight.setPower(turnPower);
            backLeft.setPower(-turnPower);
            frontLeft.setPower(-turnPower);
        }

       /* if(armUp)
            armMotor.setPower(1);
        else
            armMotor.setPower(0);

        if(armDown)
            armMotor.setPower(-1);
        else
            armMotor.setPower(0);


        if(liftUp)
            lift.setPower(1);
        else
            lift.setPower(0);

        if(liftDown)
            lift.setPower(-1);
        else
            lift.setPower(0);


        //if(clawOpen)
        //    clawPosition += arm_speed;
        //else if(clawClose)
        /0/    clawPosition -= arm_speed;

       if(clawOpen)
           clawPosition = OPEN_SERVO;
       else if (clawClose)
           clawPosition = CLOSE_SERVO;

        //clawPosition =  Range.clip(clawPosition, clawMax, clawMin);

        claw.setPosition(clawPosition);

        telemetry.addData("Claw", "%.2f", clawPosition);*/
        telemetry.addData("ForwardPower", "%.2f", forwardPower);
        telemetry.addData("TurnPower", "%.2f", turnPower);
        telemetry.update();
        //IMPORTANT claw must begin at position 0 in hardware
    }
}
