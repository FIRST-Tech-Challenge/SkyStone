package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "JustParkDepot", group = "Concept")

public class JustParkDepot extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    //motors
    DcMotor mtrFR = null;
    DcMotor mtrFL = null;
    DcMotor mtrBL = null;
    DcMotor mtrBR = null;

    //servos
    Servo grab = null;
    Servo arm = null;
    Servo extend = null;
    Servo FLservo = null;
    Servo FRservo = null;
    Servo LED_strip = null;
    DigitalChannel alliance_switch = null;
    DigitalChannel position_switch = null;


    /**

     CONSTANTS

     */

    //servos

    double FLdown = 1;
    double FRdown = 0.35;
    double FLup = 0.8;
    double FRup = 0.6;

    int strafeSwapper = 1;
    double parkingSpot = 1;
    int alliance = 0;
    int in = 0;
    int out = 1;
    private static final int red = 1;
    private static final int blue = -1;

    double LEDwhite = 0.7595;
    double LEDred = 0.6695;
    double LEDblue = 0.7445;


    double wheelDiameter = 4.0;
    double pi = 3.14159;
    //537.6 ticks per revolution for our motors
    double wheelCircumference = (wheelDiameter * pi);
    double ticksPerInch = (537.6/wheelCircumference);
    int ticksPerInchRound = 42;

    double halfPower = 0.5;
    int initialStrafe = 10*ticksPerInchRound;
    int halfPark = 22*ticksPerInchRound;
    int strafePark = 18*ticksPerInchRound;
    int finishPark = 8*ticksPerInchRound;

    int straightPark = 28*ticksPerInchRound;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());


        //motors
        mtrFR = hardwareMap.get(DcMotor.class, "rightFront_drive");
        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtrFL = hardwareMap.get(DcMotor.class, "leftFront_drive");
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtrBL = hardwareMap.get(DcMotor.class, "leftBack_drive");
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtrBR = hardwareMap.get(DcMotor.class, "rightBack_drive");
        mtrBR.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //servos
        grab = hardwareMap.get(Servo.class, "grab");
        arm = hardwareMap.get(Servo.class, "arm");
        extend = hardwareMap.get(Servo.class, "extend");
        FLservo = hardwareMap.get(Servo.class, "FL_hook");
        FLservo.setDirection(Servo.Direction.REVERSE);
        FRservo = hardwareMap.get(Servo.class, "FR_hook");
        FRservo.setDirection(Servo.Direction.REVERSE);
        LED_strip = hardwareMap.get(Servo.class, "LED_strip");

        //switches
        alliance_switch = hardwareMap.get(DigitalChannel.class, "alliance_switch");
        alliance_switch.setMode(DigitalChannel.Mode.INPUT);
        position_switch = hardwareMap.get(DigitalChannel.class, "position_switch");
        position_switch.setMode(DigitalChannel.Mode.INPUT);


        //set servos on init

        FLservo.setPosition(FLup);
        FRservo.setPosition(FRup);

        //determine side
        if (alliance_switch.getState() == true) {
            telemetry.addData("Alliance:", "Red");
            strafeSwapper = red;
            alliance = red;
        } else {
            telemetry.addData("Alliance", "Blue");
            strafeSwapper = blue;
            alliance = blue;
        }
        if (position_switch.getState() == true) {
            telemetry.addData("Innermost spot:", "### ___");
            parkingSpot = in;
        } else {
            telemetry.addData("Outer Spot", "___ ###");
            parkingSpot = out;
        }

        if (alliance == red) {
            LED_strip.setPosition(LEDred);
        }
        if (alliance == blue) {
            LED_strip.setPosition(LEDblue);
        }
        telemetry.update();

        /**

         Beginning of Program

         */

        waitForStart();

        //delay with potentiometer...potentially
        sleep(5000);

        //park
        resetEncoders();
        park(halfPower);

        /**

         End of Program

         */


    }

    private void resetEncoders() {
        mtrFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void runToPosition() {
        mtrFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mtrBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void brakeMotors() {
        mtrFL.setPower(0);
        mtrFR.setPower(0);
        mtrBL.setPower(0);
        mtrBR.setPower(0);
    }
    private void isBusy() {
        while (mtrFR.isBusy()){
        }
    }
    private void runWithoutEncoder() {
        mtrFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mtrBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void forward(double power) {
        mtrFR.setPower(power);
        mtrFL.setPower(power);
        mtrBL.setPower(power);
        mtrBR.setPower(power);
    }
    private void forwardPosition(int position) {
        mtrFR.setTargetPosition(position);
        mtrFL.setTargetPosition(position);
        mtrBR.setTargetPosition(position);
        mtrBL.setTargetPosition(position);
    }
    private void encoderForward(double power, int position){
        resetEncoders();
        forwardPosition(position);
        runToPosition();
        forward(power);
        isBusy();
        brakeMotors();
        runWithoutEncoder();
    }

    private void strafe(double power) {
        mtrFR.setPower(-power * strafeSwapper);
        mtrFL.setPower(power * strafeSwapper);
        mtrBL.setPower(-power * strafeSwapper);
        mtrBR.setPower(power * strafeSwapper);
    }
    private void strafePosition(int position){
        mtrFR.setTargetPosition(-position * strafeSwapper);
        mtrFL.setTargetPosition(position * strafeSwapper);
        mtrBR.setTargetPosition(position * strafeSwapper);
        mtrBL.setTargetPosition(-position * strafeSwapper);
    }
    private void encoderStrafe(double power, int position){
        resetEncoders();
        strafePosition(position);
        runToPosition();
        strafe(power);
        isBusy();
        brakeMotors();
        runWithoutEncoder();
    }


    private void park (double power){
        if(parkingSpot == in){
            encoderStrafe(power,initialStrafe);
            encoderForward(power,halfPark);
            encoderStrafe(power,strafePark);
            encoderForward(power,finishPark);
        }
        if(parkingSpot == out){
            encoderStrafe(power,straightPark);
        }
    }


}
