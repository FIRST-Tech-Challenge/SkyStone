package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class simpleTele extends LinearOpMode {
    private DcMotor topLeft;
    private DcMotor topRight;
    private DcMotor botLeft;
    private DcMotor botRight;

    private DcMotor winchR;
    private DcMotor winchL;
    private int liftMax;

    private DcMotor intakeR;
    private DcMotor intakeL;

    private Servo armL;
    private double armLUpperBound = 0.9;
    private double armLLowerBound = 0;
    private Servo armR;

    private Servo clawF;
    private Servo clawB;
    public boolean clawstate;

    private Servo pullL;
    private Servo pullR;
    private Servo intakeServoL;
    private Servo intakeServoR;
    public Servo capstone;

    private BNO055IMU imu;
    private DistanceSensor distance;
//    private Orientation lastAngles = new Orientation();
//    private Orientation orientation = new Orientation();

    private double globalAngle, power = .30, correction, rotation;

    private double pullRDown = 0.48;
    private double pullRUp = 0.05;
    private double pullLDown = 0.46;
    private double pullLUp = .9;

    private double ninja = 1;
    private boolean ninjaAct;
    private double currFound;
    private double currClaw;
    double timeNinja = System.currentTimeMillis();

    private boolean pushbot = false;

    public void ninja() {
        if (gamepad1.x && System.currentTimeMillis() - timeNinja > 700 && ninja == 1 && !ninjaAct) {
            ninja = 0.5;
            timeNinja = System.currentTimeMillis();
            ninjaAct = true;
        } else if (gamepad1.x && System.currentTimeMillis() - timeNinja > 700 && ninja == 0.5 && ninjaAct) {
            ninja = 1;
            timeNinja = System.currentTimeMillis();
            ninjaAct = false;
        }
    }

    public void drive(double lx, double ly, double rx) {
        topLeft.setPower(Range.clip(ninja * ly + lx + ninja * .7 * rx, -1, 1));//1.1   ; -.9
        topRight.setPower(Range.clip(ninja * ly - lx - ninja * .7 * rx, -1, 1));//-1.1  ; .9
        botLeft.setPower(Range.clip(ninja * ly - lx + ninja * .7 * rx, -1, 1));//-.9   ; 1.1
        botRight.setPower(Range.clip(ninja * ly + lx - ninja * .7 * rx, -1, 1));//.9  ; -1.1
    }

    public int positionL = 0;
    public int positionR = 0;
    public boolean init;

    public void winch(double lt, double rt) {
        if (lt > 0.1 || rt > 0.1) {
            if (!init) {
                winchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//break out of the RUNTOPOS lock
                winchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                init = true;
                locked = false;
            }
            if (winchL.getCurrentPosition() > 0 && !liftReq) {
                winchL.setPower(liftNinja * Range.clip(rt - .8*lt, -1, 1));
                winchR.setPower(liftNinja * Range.clip(rt - .8*lt, -1, 1));
                positionL = winchL.getCurrentPosition();
                positionR = winchR.getCurrentPosition();
            } else {
                winchL.setPower(liftNinja * Range.clip(rt - lt, 0, 1));
                winchR.setPower(liftNinja * Range.clip(rt - lt, 0, 1));
            }
        } else {
            if (!liftReq) {
                lock();
                init = false;
            }
            winchR.setPower(winchL.getPower());
        }
    }

    public boolean locked;

    public void lock() {
        if (!locked) {
            winchL.setTargetPosition(positionL);//lock at ended position.
            //winchR.setTargetPosition(positionR);

            winchL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //winchR.setMode(DcMotor.RunMode.RUN_TO_POSITION);//create lift lock/stall
            locked = true;
        }
    }

    public boolean foundState;

    public void pull() {
        if (gamepad1.a && !foundState && System.currentTimeMillis() > currFound + 300) {
            pullL.setPosition(pullLUp);
            pullR.setPosition(pullRUp);
            currFound = getRuntime();
            foundState = true;
            currFound = System.currentTimeMillis();
        } else if (gamepad1.a && foundState && System.currentTimeMillis() > currFound + 300) {
            pullL.setPosition(pullLDown);
            pullR.setPosition(pullRDown);
            currFound = getRuntime();
            foundState = false;
            currFound = System.currentTimeMillis();
        }
    }

    public double liftNinja = 1;
    public double clawTime = getRuntime();

    public void liftNinja() {
        if (gamepad2.x && System.currentTimeMillis() - timeNinja > 700 && liftNinja == 1) {
            liftNinja = 0.5;
            timeNinja = System.currentTimeMillis();
        } else if (gamepad2.x && System.currentTimeMillis() - timeNinja > 700 && liftNinja == 0.5) {
            liftNinja = 1;
            timeNinja = System.currentTimeMillis();
        }
    }

    //private float desiredHeadingRadians = (float) Math.toRadians(getAngle());
    private boolean called;

    //    public static final float HEADING_CORRECTION_COEFFICIENT = 0.05f;
//    public void strafe(){
//        if(!called) {
//            resetAngle();
//            desiredHeadingRadians = (float) Math.toRadians(getAngle());
//        }
//        float currentHeadingRadians = (float)Math.toRadians(getAngle());
//        float headingError = currentHeadingRadians - desiredHeadingRadians;
//        if (headingError > Math.PI) headingError -= (float)Math.PI;
//        else if (headingError < -Math.PI) headingError += (float)Math.PI;
//        correction = -HEADING_CORRECTION_COEFFICIENT * headingError;
//        if(gamepad1.dpad_left){
//            drive(-0.7, 0, correction);
//        }
//        if(gamepad1.dpad_right){
//            drive(0.7, 0, correction);
//        }
//    }
    public void claw() {
        if (gamepad2.a && !clawstate && System.currentTimeMillis() > currClaw + 300) {
            clawF.setPosition(0.8);
            clawB.setPosition(.23);
            clawTime = getRuntime();
            clawstate = true;
            currClaw = System.currentTimeMillis();
        } else if (gamepad2.a && clawstate && System.currentTimeMillis() > currClaw + 300) {
            if (armState == 0) {
                clawF.setPosition(0.42);
                clawB.setPosition(.23);
            } else {
                clawF.setPosition(0.42);
                clawB.setPosition(0.69);
            }
            clawTime = getRuntime();
            clawstate = false;
            currClaw = System.currentTimeMillis();
        }
        if (gamepad2.dpad_down && System.currentTimeMillis() > currClaw + 300) {
            clawB.setPosition(0.69);
            currClaw = System.currentTimeMillis();
        }
    }

    private boolean capState;
    private double capTime;

    public void capstone() {
        if (gamepad2.y && !capState && System.currentTimeMillis() > capTime + 300) {
            clawB.setPosition(0.69);
            sleep(200);
            capstone.setPosition(0);
//            sleep(500);
//            liftAutoDown();
            capState = true;
            capTime = System.currentTimeMillis();
        } else if (gamepad2.y && capState && System.currentTimeMillis() > capTime + 300) {
            capstone.setPosition(0.64);
            capState = false;
            capTime = System.currentTimeMillis();
        }

    }
    public boolean autoInt;
    public void intake() {
        if (gamepad1.right_bumper) {
            intakeL.setPower(1);
            intakeR.setPower(1);
            if(armState == 0)
            clawB.setPosition(.23);

        } else if (gamepad1.left_bumper) {
            if(armState == 0) {
                clawB.setPosition(0);
                if(!pushbot) {
                    clawF.setPosition(.42);
                }
            }
            intakeL.setPower(-1);
            intakeR.setPower(-1);
        } else {
            intakeL.setPower(0);
            intakeR.setPower(0);
            if(armState == 0) clawB.setPosition(.23);
        }
    }
    public void bumperIntake(){
        if(gamepad1.left_bumper || gamepad1.right_bumper) {
            autoInt = true;
        }
    }
    public void triggerIntake(double lt, double rt){
        intakeL.setPower(Range.clip(rt - lt, -1, 1));
        intakeR.setPower(Range.clip(rt - lt, -1, 1));
        if(lt != 0 || rt != 0){
            autoInt = false;
        }
    }


    private int armState = 0;
    private double armTime = System.currentTimeMillis();
    private boolean armBool = true;
    public boolean liftReq = false;

    public void arm() {
        if (gamepad2.b) {
            liftAutoDown();
        }
        if (!(winchL.isBusy() && winchR.isBusy()) && liftReq) {
            winchL.setPower(0);
            winchR.setPower(0);
            liftReq = false;
        }
        if ((gamepad2.left_bumper || gamepad2.right_bumper) && System.currentTimeMillis() > armTime + 300) {
            if (gamepad2.right_bumper) {
                armState++;
            } else {
                armState--;
            }
            armState = Range.clip(armState, 0, 4);
            if (armState == 0 && armBool) {
                armL.setPosition(.08);
                armR.setPosition(.92);
                armBool = false;
            }
            if (armState == 1 && armBool) {
                clawB.setPosition(.21);
                armL.setPosition(.5);
                armR.setPosition(.5);
                armBool = false;
            }
            if (armState == 2 && armBool) {
                armL.setPosition(.7);
                armR.setPosition(.3);
                armBool = false;
            }
            if (armState == 3 && armBool) {
                armL.setPosition(.8);
                armR.setPosition(.2);
                armBool = false;
            }
            if (armState == 4 && armBool) {
                armL.setPosition(.9);
                armR.setPosition(.1);
                armBool = false;
            }
            armTime = System.currentTimeMillis();
        }
        armBool = true;
    }

    public void intakeServo() {
        if (gamepad1.dpad_down) {
            intakeServoL.setPosition(0.1);
            intakeServoR.setPosition(.85);
        }
    }

    private double pushTime = System.currentTimeMillis();

    public void setPushbot() {
//        if ((gamepad1.dpad_up || gamepad2.dpad_up) && System.currentTimeMillis() > pushTime + 300) {
//            if (!pushbot) {
//                pushbot = true;
//                clawF.setPosition(1);
//            } else {
//                pushbot = false;
//                clawF.setPosition(0.42);
//            }
//            pushTime = System.currentTimeMillis();
//        }
    }

    public void liftAutoDown() {
        liftReq = true;
        if (armState == 2 || armState == 3 || armState == 4) {
            clawF.setPosition(.42);
            clawB.setPosition(.69);
            sleep(200);
            winchL.setTargetPosition(winchL.getCurrentPosition() + 200);
            winchR.setTargetPosition(winchR.getCurrentPosition() + 200);
            if (winchL.getCurrentPosition() > 0) {
                winchL.setPower(1);
                winchR.setPower(1);

                winchL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                winchR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            armL.setPosition(.5);
            armR.setPosition(.5);
            sleep(100);
            clawB.setPosition(.21);
            sleep(400);

            armL.setPosition(.08);
            armR.setPosition(.92);
            armState = 0;
            winchL.setTargetPosition(0);
            winchR.setTargetPosition(0);
            if (winchL.getCurrentPosition() > 0) {
                winchL.setPower(-1);
                winchR.setPower(-1);
            }
            winchL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            winchR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        liftReq = false;
    }

    public void autoIntake() {
        intakeL.setPower(1);
        intakeR.setPower(1);
    }

    public void autoIntakeOff() {
        intakeL.setPower(0);
        intakeR.setPower(0);
    }


    public void grip() {
        clawF.setPosition(0.8);
        clawB.setPosition(0.21);
    }

    public void release() {

        if (armState > 0) {
            clawF.setPosition(0.42);
            clawB.setPosition(0.69);
        } else {
            clawF.setPosition(.42);
            clawB.setPosition(.21);
        }

    }

    public boolean grabbed = false;

    public void autoGrip(){

        if (armState != 0 || winchL.getCurrentPosition() > 100) return;

        if (distance.getDistance(DistanceUnit.CM) < 10 && !grabbed) {
            //autoIntakeOff();
            grip();
            telemetry.addData("Gripped", "");
            telemetry.update();
            grabbed = true;
        } else if (!(distance.getDistance(DistanceUnit.CM) < 10) && grabbed) {
            //autoIntake();
            telemetry.addData("No Stone", "");
            telemetry.update();
            grabbed = false;
        }
    }



//    public void run() {
//        boolean grabbed = false;
//        while (opModeIsA
//        ctive()) {
//            if (distance.getDistance(DistanceUnit.CM) < 10 && !grabbed) {
//                autoIntakeOff();
//                grip();
//                grabbed = true;
//            }
//
//            else if(!(distance.getDistance(DistanceUnit.CM) < 10) && grabbed){
//                autoIntake();
//                release();
//                grabbed = false;
//            }
////            if (armState != 0 || winchL.getCurrentPosition() < 100) continue;
////
////            if (distance.getDistance(DistanceUnit.CM) < 10 && !grabbed) {
////                autoIntakeOff();
////                grip();
////                telemetry.addData("Gripped", "");
////                telemetry.update();
////                grabbed = true;
////            } else if (!(distance.getDistance(DistanceUnit.CM) < 10) && !grabbed) {
////                autoIntake();
////                grabbed = false;
////            }
//        }
//    }
//    private void resetAngle()//for PID
//    {
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        globalAngle = 0;
//    }
//    public double getAngle()//for PID
//    {
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
//
//        if (deltaAngle < -180)
//            deltaAngle += 360;
//        else if (deltaAngle > 180)
//            deltaAngle -= 360;
//
//        globalAngle += deltaAngle;
//
//        lastAngles = angles;
//
//        return globalAngle;
//    }


    @Override
    public void runOpMode() throws InterruptedException {
        topLeft = hardwareMap.dcMotor.get("topLeft");
        topRight = hardwareMap.dcMotor.get("topRight");
        botLeft = hardwareMap.dcMotor.get("botLeft");
        botRight = hardwareMap.dcMotor.get("botRight");

        winchR = hardwareMap.dcMotor.get("winchR");
        winchL = hardwareMap.dcMotor.get("winchL");

        intakeL = hardwareMap.dcMotor.get("intakeL");
        intakeR = hardwareMap.dcMotor.get("intakeR");

        pullL = hardwareMap.servo.get("pullL");
        pullR = hardwareMap.servo.get("pullR");

        armL = hardwareMap.servo.get("armL");
        armR = hardwareMap.servo.get("armR");

        clawF = hardwareMap.servo.get("clawF");
        clawB = hardwareMap.servo.get("clawB");
        capstone = hardwareMap.servo.get("capstone");

        intakeServoL = hardwareMap.servo.get("intakeServoL");
        intakeServoR = hardwareMap.servo.get("intakeServoR");

        distance = hardwareMap.get(DistanceSensor.class, "distance");

//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled = false;
//
//        imu.initialize(parameters);
//        orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        winchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        winchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        botLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        topRight.setDirection(DcMotorSimple.Direction.FORWARD);
        botRight.setDirection(DcMotorSimple.Direction.FORWARD);

        winchR.setDirection(DcMotorSimple.Direction.FORWARD);
        winchL.setDirection(DcMotorSimple.Direction.FORWARD);
        winchL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeL.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeR.setDirection(DcMotorSimple.Direction.REVERSE);

//        simpleTele s = new simpleTele();
//        Thread t = new Thread(s);
        waitForStart();

        //t.start();

        armL.setPosition(0.08);
        armR.setPosition(0.92);
        clawF.setPosition(0.42);
        clawB.setPosition(.21);
        pullL.setPosition(pullLUp);
        pullR.setPosition(pullRUp); //

        capstone.setPosition(0.64);

        while (opModeIsActive()) {
            if (!(gamepad1.dpad_left && gamepad1.dpad_right)) {
                called = false;
                drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            }
            intake();
            intakeServo();
            pull();
            ninja();
            autoGrip();
            //strafe();
            if (!pushbot) {
                capstone();
                liftNinja();
                if (!liftReq)
                    winch(gamepad2.left_trigger, gamepad2.right_trigger);
                arm();
                claw();
            }
            setPushbot();
            idle();
        }
    }
    
        
}
