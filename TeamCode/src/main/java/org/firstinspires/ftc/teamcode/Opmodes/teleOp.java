package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Tele-Op", group="Linear Opmode")

public class teleOp extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor driveFrontLeft;
    private DcMotor driveFrontRight;
    private DcMotor driveBackLeft;
    private DcMotor driveBackRight;
    double m1, m2, m3, m4;
    double x1, x2, y1, y2, s1, s2, s3;

    private DcMotor liftRight;
    private DcMotor liftLeft;
    private boolean liftReturning;

    private DcMotor intakeLeft;
    private DcMotor intakeRight;

    private DigitalChannel leftLiftLimitSwitch;
    private DigitalChannel rightLiftLimitSwitch;

    private Servo foundationClampLeft;
    private Servo foundationClampRight;

    private Servo flipperServoRight;
    private Servo flipperServoLeft;

    private Servo blockGrabberFront;
    private Servo blockGrabberBack;

    private Servo autoFlipperLeft;
    private Servo autoFlipperRight;
    private Servo autoGrabberLeft;
    private Servo autoGrabberRight;

    private String liftState = "resting";
    private double miles1 = 0;
    private double milesHoldHeight = 0;

    private int lastPressedFlipper = 0; //0 is in position , 1 is out position
    private int liftAverage = 0;
    private int liftMax = 99999;
    private int liftGoal = 0;
    private int blocklevel = 0; //first level corresponds to second block. 0th level is first block.

    private boolean dpadUpPressed = false;

    private boolean liftAtBottom = true;
    private int grabberState = 0;

    private int liftLowerTimer = 0;

    private  int dropoffCounter = 0;
    private int dropoffslidePosition = 0;
    private int slideSafetyCount = 0;

    private void initialize() {
        // Initialize all objects declared above
        //MOTOR NAMING SCHEME FOR HARDWARE MAP:
        //Motors should be named as [motor function] + [motor position]
        //Ie a drive motor in front left position is named "driveFrontLeft"
        driveFrontRight = hardwareMap.dcMotor.get("driveFrontRight"); //done
        driveFrontLeft = hardwareMap.dcMotor.get("driveFrontLeft");
        driveBackLeft = hardwareMap.dcMotor.get("driveBackLeft");
        driveBackRight = hardwareMap.dcMotor.get("driveBackRight");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        liftRight = hardwareMap.dcMotor.get("liftRight");
        liftLeft = hardwareMap.dcMotor.get("liftLeft");


        //SERVOS FROM HARDWARE MAP:
        //Use same naming scheme as motors when available, otherwise, use a logical name
        blockGrabberFront = hardwareMap.servo.get("blockGrabberFront");
        blockGrabberBack = hardwareMap.servo.get("blockGrabberBack");

        flipperServoLeft = hardwareMap.servo.get("flipperServoLeft");
        flipperServoRight = hardwareMap.servo.get("flipperServoRight");

        //this servo (intakeDropper) is continous rotation
        //intakeDropper = hardwareMap.servo.get("intakeDropper");

        foundationClampLeft = hardwareMap.servo.get("foundationClampLeft");
        foundationClampRight = hardwareMap.servo.get("foundationClampRight");

        //auto servos:
        autoFlipperLeft = hardwareMap.servo.get("autoFlipperLeft");
        autoFlipperRight = hardwareMap.servo.get("autoFlipperRight");
        autoGrabberLeft = hardwareMap.servo.get("autoGrabberLeft");
        autoGrabberRight = hardwareMap.servo.get("autoGrabberRight");

        //capstone = hardwareMap.servo.get("capstone");


        leftLiftLimitSwitch = hardwareMap.digitalChannel.get("leftLiftLimitSwitch");
        rightLiftLimitSwitch = hardwareMap.digitalChannel.get("rightLiftLimitSwitch");

        //Some Housekeeping========================================================================
        leftLiftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rightLiftLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackLeft.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        //Lift Housekeeping
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized - Welcome, Operators");
        telemetry.update();

    }

    @Override
    public void runOpMode() {
        // Wait for the game to start (driver presses PLAY)
        initialize();
        waitForStart();
        telemetry.update();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status: ", "Systems nominal - Good Luck, Operators");
            telemetry.addData("LiftAverage: ", liftAverage); //important value
            telemetry.addData("LiftGoal: ", liftGoal);
            telemetry.addData("LiftAtBottom: ", liftAtBottom);
            telemetry.update();


            //DRIVE=================================================================================

            y2 = -gamepad1.left_stick_y;
            y1 = -gamepad1.right_stick_y;
            x1 = gamepad1.right_stick_x;
            x2 = gamepad1.left_stick_x;

            if (gamepad1.right_bumper) {
                s1 = 0.4;
            } else {
                s1 = 1.0;
            }
            s2 = 1;


            m1 = (y1 - x1) * s1 * s2;
            m2 = (y1 + x1) * s1 * s2;
            m3 = (-y2 - x2) * s1 * s2;
            m4 = (-y2 + x2) * s1 * s2;


            driveFrontLeft.setPower(m3);    driveFrontRight.setPower(m1);
            driveBackLeft.setPower(m4);     driveBackRight.setPower(m2);

//VERTICAL EXTRUSION=============================================================================-
            //UNDERPOWRRED
            ///*

            liftAverage = -(liftLeft.getCurrentPosition() + liftRight.getCurrentPosition()) / 2;
            if (!leftLiftLimitSwitch.getState() || !rightLiftLimitSwitch.getState()) {
                liftAtBottom = true;
                slideSafetyCount = 0;
            }else{
                liftAtBottom = false;
            }
            //STATE PICKER: choose which section of code to run depending on what is pressed for this game tick.
            if(liftAtBottom && dropoffCounter == 0){
                liftState = "resting";
                blocklevel = 0;
                milesHoldHeight = 0;
            }
            if(gamepad2.dpad_up || gamepad2.dpad_down){
                liftState = "holdingTarget";
            }
            if(gamepad2.dpad_left || gamepad2.dpad_right){
                liftState = "freefall";
            }
            if(gamepad2.a && slideSafetyCount>200){
                if(slideSafetyCount<100){
                    liftState = "retracting";

                }
                else{
                    liftState = "resting";

                }
            }
            if(gamepad2.right_bumper){
                liftState = "droppingOff";
            }
            if(gamepad1.right_bumper){
                liftState = "capstoneDroppingOff";
            }




            //STATE EXECUTER: runs a certain chunk of code, depending on the last button pressed by operator.
            if (liftState.equals("resting")){
                //if at the bottom, and no new controls given... just do nothing!
                liftLeft.setPower(0);
                liftRight.setPower(0); //set motor power to zero if you ever hit the bottom (2/3/2020 update)
                if (dpadUpPressed){ //(2/5/2020 edit), sends code to "holdingTarget" if user presses Dpad. So it doesn't get stuck in "resting"
                    liftState = "holdingTarget";
                }

            }
            if (liftState.equals("holdingTarget")){
                dropoffCounter = 0; //(2/13/2020 change)
                if (gamepad2.dpad_up) {
                    dpadUpPressed = true;
                }
                if (!gamepad2.dpad_up && dpadUpPressed) { //code handles the user-input




                    if (blocklevel == 0) { liftGoal = 185; }
                    if (blocklevel == 1) { liftGoal =420; }
                    if (blocklevel == 2) { liftGoal = 655; }
                    if (blocklevel == 3) { liftGoal = 871; }
                    if (blocklevel == 4) { liftGoal = 1067; }
                    if (blocklevel == 5) { liftGoal = 1261; }
                    if (blocklevel == 6) { liftGoal = 1465; } // if (blocklevel == 7){liftGoal = liftGoal + }  if (blocklevel == 8){liftGoal = liftGoal + }
                    if (blocklevel == 7) { liftGoal = 1690; }
                    if (blocklevel == 8) { liftGoal = 1900; }
                    if (blocklevel == 9) { liftGoal = 2110; }




                    blocklevel = blocklevel + 1;
                    lastPressedFlipper = 2; //sets the flipper to upright position when dpadUp is pressed.
                    if (liftGoal > liftMax) {
                        liftGoal = liftMax; //makes sure the slides don't exceed the max encoder position.
                    }
                    dpadUpPressed = false;
                }
                if (liftGoal - liftAverage > 100 ) { //code handles the robot (sets motor powers for lift)
                    liftRight.setPower(-1.0);
                    liftLeft.setPower(-1.0);
                }
                else if (liftGoal - liftAverage > 15 && liftGoal - liftAverage < 100 ) {
                    liftRight.setPower(-0.2);
                    liftLeft.setPower(-0.2);
                }
                else if (liftGoal - liftAverage > -15 && liftGoal - liftAverage < 15 ) { //set to drift between -15 and +15 encoder ticks from target
                    liftRight.setPower(-0.02); //hover power for holdingTarget mode
                    liftLeft.setPower(-0.02);
                }
                else if (liftGoal - liftAverage < -15 ) {
                    liftRight.setPower(0);
                    liftLeft.setPower(0);
                }


            }
            if (liftState.equals("droppingOff")){//this is the auto dropoff sequence state
                if(dropoffCounter ==0){
                    dropoffslidePosition = liftAverage + 300; //(2/5/20 edit), changed from +100 to +175 to make lift go higher on dropoff sequence
                }
                dropoffCounter = dropoffCounter + 1; //increment dropoff counter - this keeps track of time.

                if(dropoffCounter < 3) { //first loop of code, open the servo and find a target slide height
                    grabberState = 1;
                }
                if (dropoffslidePosition > liftAverage && liftAverage < liftMax && dropoffCounter > 10){ //(2/5/2020 edit) - pauses for a bit (dropoff counter > 3) before powering slides
                    liftLeft.setPower(-.7);
                    liftRight.setPower(-.7);
                }
                if (dropoffCounter > 20 && (dropoffslidePosition - liftAverage) < 50){ //after 12 loops, the flipper servos will start moving into the robot.
                    lastPressedFlipper = 0; //changes flipper variable so the flipper is set to inside chassis pos
                    grabberState = 0; //sets the grabbers to rear closed, front open state

                }
                if (dropoffCounter > 34 && (dropoffslidePosition - liftAverage) < 50){ //after 12 loops, the flipper servos will start moving into the robot.
                    lastPressedFlipper = 0; //changes flipper variable so the flipper is set to inside chassis pos
                    grabberState = 0; //sets the grabbers to rear closed, front open state
                    dropoffCounter = 0; //sequence is over, so reset counter variable
                    liftState = "retracting"; //pass the robot over to the 'retracting' code for A-reset
                }
            }
            if (liftState.equals("capstoneDroppingOff")){//this is the auto dropoff sequence state
                if(dropoffCounter ==0){
                    dropoffslidePosition = liftAverage + 300; //(2/5/20 edit), changed from +100 to +175 to make lift go higher on dropoff sequence
                }
                dropoffCounter = dropoffCounter + 1; //increment dropoff counter - this keeps track of time.

                if(dropoffCounter < 3) { //drops off capstone by setting servo to position 3.
                    grabberState = 3;
                }

                if (dropoffslidePosition > liftAverage && liftAverage < liftMax && dropoffCounter > 20){ //(2/5/2020 edit) - pauses for a bit (dropoff counter > 3) before powering slides
                    liftLeft.setPower(-.7);
                    liftRight.setPower(-.7);
                }
                if (dropoffCounter > 48 && (dropoffslidePosition - liftAverage) < 50){ //after 12 loops, the flipper servos will start moving into the robot.
                    lastPressedFlipper = 0; //changes flipper variable so the flipper is set to inside chassis pos
                    grabberState = 0; //sets the grabbers to rear closed, front open state

                }
                if (dropoffCounter > 62& (dropoffslidePosition - liftAverage) <50){ //after 12 loops, the flipper servos will start moving into the robot.
                    lastPressedFlipper = 0; //changes flipper variable so the flipper is set to inside chassis pos
                    grabberState = 0; //sets the grabbers to rear closed, front open state
                    dropoffCounter = 0; //sequence is over, so reset counter variable
                    liftState = "retracting"; //pass the robot over to the 'retracting' code for A-reset
                }
            }
            if (liftState.equals("retracting")){
                dropoffCounter = 0;//(2/13/2020 change)
                if (liftAverage > 200) {
                    liftRight.setPower(0.7);
                    liftLeft.setPower(0.7);
                    liftLowerTimer = 0;
                }
                if ( liftAverage < 200) {
                    liftLowerTimer = liftLowerTimer + 1;
                    slideSafetyCount = slideSafetyCount + 1;
                    if (liftLowerTimer < 2) {
                        lastPressedFlipper = 0; //(2/5/2020 edit) set servo positions. This loop only runs twice or so
                        grabberState = 0; //
                        liftRight.setPower(-.075); //this is really agressive. IDK if its necessary. Also may be dangerous for motors, battery, fuse, and string breaking. IDK.
                        liftLeft.setPower(-.075);
                    } else {
                        liftRight.setPower(0.2);
                        liftLeft.setPower(0.2);
                    }
                }
                if (liftAtBottom == true){ //set motors to zero after hitting the bottom (2/3/2020 update)
                    liftState = "resting";
                    blocklevel = 0; //(2/13/2020 change)
                }

            }
            if (liftState.equals("freefall")){ //equivalent of "manual mode"
                dropoffCounter = 0;//(2/13/2020 change)
                if (liftAtBottom == true){
                    liftState = "resting";
                    blocklevel = 0; //(2/13/2020 change)
                }

                if(gamepad1.left_trigger > 0.2){
                    if (milesHoldHeight == 0){
                        milesHoldHeight = liftAverage; //find the hold height
                    }
                    if (liftAverage > milesHoldHeight + 20 ){//(2/5/20 edit), changed +10 to +20 to allow more encoder variance (motors are on more of the time)
                        miles1 = 0;
                    }else{
                        miles1 = -.02;
                    }

                }else{
                    miles1 = 0; //added to make sure this doesn't stay engaged. (2/3/2020 update)
                }

                if(gamepad2.dpad_left && liftAverage < liftMax){ //raise lift
                    liftLeft.setPower(-.6 + miles1);
                    liftRight.setPower(-.6 + miles1);//typo (2/3/2020 update)
                }else if (gamepad2.dpad_right && !liftState.equals("resting")){ //lower lift, unless it's fully lowered
                    liftLeft.setPower(.4 + miles1);
                    liftRight.setPower(.4 + miles1);//typo (2/3/2020 update)
                }
                else{
                    liftLeft.setPower(0 + miles1);
                    liftRight.setPower(0 + miles1);//typo (2/3/2020 update)
                }
            }


//BLOCK GRIPPER====================================================================================


            if (gamepad2.b) {
                grabberState = 0;
                // Receive position back grabber down and front grabber up
            }
            if (gamepad2.x) {
                grabberState = 1;
                // deposit position both grabbers are up
            }
            if (gamepad2.y) {
                grabberState = 2;
                // clamp position both grabber are down
            }
          /* if (gamepad1.x) {
               grabberState = 3;

           }
*/

            if (grabberState == 0 && !gamepad1.x) {
                blockGrabberFront.setPosition(0.35);
                blockGrabberBack.setPosition(0.7);
            }
            if (grabberState == 1 && !gamepad1.x) {
                blockGrabberBack.setPosition(0.35);
                blockGrabberFront.setPosition(0.35);
            }
            if (grabberState == 2 && !gamepad1.x) {
                blockGrabberFront.setPosition(0.7); //note: for gobilda servos, increase turns servo Clockwise
                blockGrabberBack.setPosition(0.7);//Clockwise: viewing from infront servo, track direction of arm
            }
            if (grabberState == 3 || gamepad1.x) {
                blockGrabberFront.setPosition(0.9); //note: for gobilda servos, increase turns servo Clockwise
                blockGrabberBack.setPosition(0.9);//Clockwise: viewing from infront servo, track direction of arm

            }
            if (grabberState == 3 || gamepad1.x) {
                blockGrabberFront.setPosition(0.9); //note: for gobilda servos, increase turns servo Clockwise
                blockGrabberBack.setPosition(0.9);//Clockwise: viewing from infront servo, track direction of arm
            }


            //BLOCK FLIPPER=====================================================================================

            if (gamepad2.left_trigger > 0.2) {
                lastPressedFlipper = 0; //0 is in the chassis, 1 is out of the chassis

            }
            if (gamepad2.left_bumper) {
                lastPressedFlipper = 1; //0 is in the chassis, 1 is out of the chassis
            }
            if (lastPressedFlipper == 0) {
                flipperServoLeft.setPosition(0.25);//.31
                flipperServoRight.setPosition(0.75);//.69
            } else if (lastPressedFlipper == 1) {
                flipperServoLeft.setPosition(0.90);//.89
                flipperServoRight.setPosition(0.10);//.11
            } else if (lastPressedFlipper == 2) { //middle position. Lift Code sets the variable to 2.
                flipperServoLeft.setPosition(0.35);//.89
                flipperServoRight.setPosition(0.65);//.11
            }




            //INTAKE ===========================================================================================

            intakeLeft.setPower(-gamepad2.left_stick_y * 1.0);
            intakeRight.setPower(gamepad2.right_stick_y * 1.0);
//            intakeLeft.setPower(-1.0);
//            intakeRight.setPower(1.0);


            //FOUNDATION CLAMP--=============================================================

            if (gamepad1.left_bumper) {
                foundationClampLeft.setPosition(.190);
                foundationClampRight.setPosition(.81);
            } else {
                foundationClampLeft.setPosition(0.745);
                foundationClampRight.setPosition(0.26);
            }

            //AUTO BLOCK GRABBERS=============================================================
            if (gamepad1.a) {
                autoFlipperRight.setPosition(0.0); //flipper down
                autoFlipperLeft.setPosition(0.036); //put arm fully down
            } else {
                autoFlipperRight.setPosition(0.4); //flipper up
                autoFlipperLeft.setPosition(0.196);
            }
            if (gamepad1.y) {
                autoGrabberRight.setPosition(0.0); //grabbers open
                autoGrabberLeft.setPosition(0.64);
            } else {
                autoGrabberRight.setPosition(0.286); //grabbers closed
                autoGrabberLeft.setPosition(0.281); //grabbers closed
            }

            //CAPSTONE==========================================================================
/*
         if (gamepad1.right_trigger > 0.2) {
             capstone.setPosition(.25); //capstone down
         } else {
             capstone.setPosition(.68);//capstone up
         }

*/

        }
    }

}