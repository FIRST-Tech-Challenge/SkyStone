package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FoundationMovePark", group = "Concept")

public class FoundationMovePark extends LinearOpMode {

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
    double groundArm = 0.05;
    double retractArm = 0.75;

    double fullGrab = 0.33;
    double releaseGrab = 0.7;

    double FLdown = 1;
    double FRdown = 0.27;
    double FLup = 0.8;
    double FRup = 0.57;

    int strafeSwapper = 1;
    double parkingSpot = 1;
    int alliance = 1;
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

    double lowestPower = 0.1;
    double lowPower = 0.3;
    double halfPower = 0.5;
    double fullPower = 1;
    int initialStrafe = 10*ticksPerInchRound;
    int wallToFoundation = 26*ticksPerInchRound;
    int foundationSlow = 7*ticksPerInchRound;
    int backUpWithFoundation = 10*ticksPerInchRound;
    int turnPull = 55*ticksPerInchRound;
    int pushAgainstWall = 18*ticksPerInchRound;
    int backUpFromFoundation = 8*ticksPerInchRound;
    int strafeToWall = 17*ticksPerInchRound;
    int straightPark = 30*ticksPerInchRound;
    int halfPark = 9*ticksPerInchRound;
    int finishPark = 21*ticksPerInchRound;
    int strafePark = 27*ticksPerInchRound;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());


        //motors
        mtrFR = hardwareMap.get(DcMotor.class, "rightFront_drive");
        mtrFR.setDirection(DcMotor.Direction.REVERSE);
        mtrFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mtrFL = hardwareMap.get(DcMotor.class, "leftFront_drive");
        mtrFL.setDirection(DcMotor.Direction.FORWARD);
        mtrFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mtrBL = hardwareMap.get(DcMotor.class, "leftBack_drive");
        mtrBL.setDirection(DcMotor.Direction.FORWARD);
        mtrBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mtrBR = hardwareMap.get(DcMotor.class, "rightBack_drive");
        mtrBR.setDirection(DcMotor.Direction.REVERSE);
        mtrBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        }
        else {
            telemetry.addData("Alliance", "Blue");
            strafeSwapper = blue;
            alliance = blue;
        }
        if (position_switch.getState() == true) {
            telemetry.addData("Innermost spot:", "### ___");
            parkingSpot = in;
        }
        else {
            telemetry.addData("Outer Spot", "___ ###");
            parkingSpot = out;
        }

        if(alliance == red) {
            LED_strip.setPosition(LEDred);
        }
        if(alliance == blue) {
            LED_strip.setPosition(LEDblue);
        }
        telemetry.update();

        /**

         Beginning of Program

         */

        waitForStart();
        //strafe over a little to get in position
        encoderStrafe(lowPower,initialStrafe);

        //drive up to foundation
        encoderForward(halfPower,wallToFoundation);
        encoderForward(lowestPower,foundationSlow);

        //lower hooks to grab foundation
        lowerHooks();
        waitFor(0.5);

        //back up with foundation
        encoderForwardTime(-lowPower,-backUpWithFoundation,3);

        //turn the foundation
        encoderTurnPull();

        //push foundation against wall
        LED_strip.setPosition(LEDblue);
        encoderForward(halfPower,pushAgainstWall);

        //lift up hooks
        raiseHooks();

        //back off from foundation a bit
        encoderForward(-0.3,-backUpFromFoundation);

        //strafe right into wall
        encoderStrafe(lowPower,strafeToWall);

        //park
        park(halfPower);

        /**

         End of Program

         */

    }

    private void waitFor(double waittime) {
        runtime.reset();
        while (runtime.time() < waittime) {
        }
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
    private void mtrFRisBusy() {
        while (mtrFR.isBusy()){
        }
    }
    private void mtrBLisBusy() {
        while (mtrBL.isBusy()){
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
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }
    private void encoderForwardTime(double power, int position, double time){
        resetEncoders();
        forwardPosition(position);
        runToPosition();
        forward(power);
        while (mtrFR.isBusy()) {
            if(runtime.time()>time){
                break;
            }
        }
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
        mtrFRisBusy();
        brakeMotors();
        runWithoutEncoder();
    }

    private void turnPullPower(double power){
        if(alliance == red){
            mtrFL.setPower(power);
            mtrBL.setPower(0);
            mtrFR.setPower(-(power+0.1));
            mtrBR.setPower(-(power+0.2));
        }
        else{
            mtrFL.setPower(-(power+0.1));
            mtrBL.setPower(-(power+0.2));
            mtrFR.setPower(power);
            mtrBR.setPower(0);
        }
    }

    private void turnPullRed(int position){
        //lime green
        LED_strip.setPosition(0.7045);
        mtrFR.setTargetPosition(-position);
        mtrFL.setTargetPosition(8*ticksPerInchRound);
        mtrBR.setTargetPosition(-position);
        mtrBL.setTargetPosition(0);
        //mtrBL.setTargetPosition(-(position - (16 * ticksPerInchRound)));
        runToPosition();
        turnPullPower(halfPower);
    }
    private void turnPullBlue(int position){
        //white
        LED_strip.setPosition(0.7595);
        mtrFR.setTargetPosition(8*ticksPerInchRound);
        mtrFL.setTargetPosition(-position);
        mtrBR.setTargetPosition(0);
        //mtrBR.setTargetPosition(-(position - (16 * ticksPerInchRound)));
        mtrBL.setTargetPosition(-position);
        runToPosition();
        turnPullPower(halfPower);
    }
    private void encoderTurnPull(){
        resetEncoders();
        if(alliance == red){
            telemetry.addData("Turning", "Red");
            telemetry.update();
            turnPullRed(turnPull);
            mtrFRisBusy();
        }
        else{
            telemetry.addData("Turning", "Blue");
            telemetry.update();
            resetEncoders();
            turnPullBlue(turnPull);
            mtrBLisBusy();
        }
        brakeMotors();
        runWithoutEncoder();
    }

    private void lowerHooks(){
        FLservo.setPosition(FLdown);
        FRservo.setPosition(FRdown);
    }
    private void raiseHooks(){
        FLservo.setPosition(FLup);
        FRservo.setPosition(FRup);
    }

    private void park(double power){
        if(parkingSpot == in){
            encoderForward(-power,-halfPark);
            encoderStrafe(-power,-strafePark);
            encoderForward(-power,-finishPark);
        }
        if(parkingSpot == out){
            encoderForward(-power, -straightPark);
        }
    }
}
