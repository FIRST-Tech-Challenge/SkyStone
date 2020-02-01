package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="teleop new controls", group="Linear Opmode")
//@Disabled
public class teleop_new_controls extends LinearOpMode {


    /**



     ~ time to make ben suffer ~



     */


    ElapsedTime runtime = new ElapsedTime();

    //motors
    DcMotor mtrBL;
    DcMotor mtrBR;
    DcMotor mtrFL;
    DcMotor mtrFR;
    DcMotor mtrLift;

    //servos
    Servo extend;
    Servo grab;
    Servo arm;
    Servo FRservo;
    Servo FLservo;
    Servo LED_strip;

    //sensors
    DistanceSensor sensorRange;

    //limit switches
    TouchSensor topLimit;
    TouchSensor bottomLimit;

    /**

     CONSTANTS

     */

    //servos
    double extendOut = 0.75;
    double extendIn = 0;

    double groundArm = 0.05;
    double foundArm = 0.18;
    double foundSecure = 0.1;
    double highArm = 0.36;
    double highSecure = 0.29;
    double retractArm = 0.75;

    double fullGrab = 0.33;
    double releaseGrab = 0.7;

    double FLdown = 1;
    double FLup = 0.8;
    double FRdown = 0.27;
    double FRup = 0.53;

    //motors
    double liftReversal = 0.05;
    double liftStop = -0;

    double slow = 1;

    double LEDcolor = 0.2375;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // motors
        mtrBL = hardwareMap.get(DcMotor.class, "leftFront_drive");
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBL.setDirection(DcMotor.Direction.REVERSE);

        mtrBR = hardwareMap.get(DcMotor.class, "rightBack_drive");
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrBR.setDirection(DcMotor.Direction.FORWARD);

        mtrFL = hardwareMap.get(DcMotor.class, "leftBack_drive");
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFL.setDirection(DcMotor.Direction.REVERSE);

        mtrFR = hardwareMap.get(DcMotor.class, "rightFront_drive");
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrFR.setDirection(DcMotor.Direction.FORWARD);

        mtrLift = hardwareMap.get(DcMotor.class, "lift_power");
        mtrLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mtrLift.setDirection(DcMotor.Direction.REVERSE);

        //servos
        extend = hardwareMap.get(Servo.class, "extend");
        extend.setDirection(Servo.Direction.FORWARD);

        arm = hardwareMap.get(Servo.class, "arm");
        arm.setDirection(Servo.Direction.FORWARD);

        grab = hardwareMap.get(Servo.class, "grab");
        grab.setDirection(Servo.Direction.REVERSE);

        FLservo = hardwareMap.get(Servo.class, "FL_hook");
        FLservo.setDirection(Servo.Direction.REVERSE);

        FRservo = hardwareMap.get(Servo.class, "FR_hook");
        FRservo.setDirection(Servo.Direction.REVERSE);

        //LED lights
        LED_strip = hardwareMap.get(Servo.class, "LED_strip");

        //sensors
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        //limit switches
        topLimit = hardwareMap.get(TouchSensor.class, "topLimit");
        bottomLimit = hardwareMap.get(TouchSensor.class, "bottomLimit");

        //set on init
        LED_strip.setPosition(LEDcolor);
        grab.setPosition(fullGrab);
        FLservo.setPosition(FLup);
        FRservo.setPosition(FRup);

        //start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            /**

             GAMEPAD 1 CONTROLS

             **/

            //DRIVING
            //Driving: left stick y is forward and backwards and left stick x is turning.
            //right stick x strafes.
            mtrBL.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x - gamepad1.left_stick_x);
            mtrBR.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x - gamepad1.left_stick_x);
            mtrFL.setPower(gamepad1.left_stick_y - gamepad1.right_stick_x + gamepad1.left_stick_x);
            mtrFR.setPower(gamepad1.left_stick_y + gamepad1.right_stick_x + gamepad1.left_stick_x);

            //GRABBING
            //full grab
            if(gamepad1.right_trigger == 1) {
                grab.setPosition(fullGrab);
            }
            //release grab
            if(gamepad1.left_trigger == 1) {
                grab.setPosition(releaseGrab);
            }


            //FOUNDATION MOVEMENT
            //claws both go down
            if(gamepad1.dpad_up) {
                FLservo.setPosition(FLdown);
                FRservo.setPosition(FRdown);
            }
            //claws both retract
            if(gamepad1.dpad_down){
                FLservo.setPosition(FLup);
                FRservo.setPosition(FRup);
            }




            /**

             GAMEPAD 2 CONTROLS

             **/

            //ARM EXTENSION
            //extend arm for grabbing on the playing field (not foundation)
            if (gamepad2.a) {
                arm.setPosition(groundArm);
            }
            //halfway hover with block (used for placing higher than first level)
            if (gamepad2.y) {
                arm.setPosition(highArm);
            }
            //secure the halfway placing
            if (gamepad2.left_bumper) {
                arm.setPosition(highSecure);
            }
            //foundation hover with block
            if (gamepad2.x) {
                arm.setPosition(foundArm);
            }
            //secure foundation placing
            if (gamepad2.b) {
                arm.setPosition(foundSecure);
            }
            //retract the arm
            if (gamepad2.right_bumper) {
                arm.setPosition(retractArm);
                sleep(250);
                grab.setPosition(fullGrab);
            }

            if (gamepad2.dpad_down){
                extend.setPosition(extendIn);
            }
            if (gamepad2.dpad_up){
                extend.setPosition(extendOut);
            }

            //GRABBING
            //full grab
            if(gamepad2.right_trigger == 1) {
                grab.setPosition(fullGrab);
            }
            //release grab
            if(gamepad2.left_trigger == 1) {
                grab.setPosition(releaseGrab);
            }

            //LIFT
            //top limit switch
            if(gamepad2.left_stick_y<0 && topLimit.isPressed()){
                mtrLift.setPower(liftReversal);
            }
            else {
                //bottom limit switch
                if(gamepad2.left_stick_y>0 && bottomLimit.isPressed()){
                    mtrLift.setPower(liftStop);
                }
                else {mtrLift.setPower(gamepad2.left_stick_y);}
            }

            //shows elapsed time and limit switch values
            telemetry.addData("Status: ","Ben's suffering 100");
            telemetry.addData("Top Limit state", "State: " + topLimit.isPressed());
            telemetry.addData("Bottom Limit state", ": " + bottomLimit.isPressed());
            telemetry.update();
        }
    }
}


