package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.util.Range;



import java.lang.Math;

@TeleOp(name = "MechTeleOp")
public class MechTeleOp extends OpMode {
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    final double calibFL = 1.00f;
    final double calibFR = 1.00f;
    final double calibBL = 1.00f;
    final double calibBR = 1.00f;

    DcMotor armOne;
    DcMotor armTwo;
    DcMotor armThree;


    double[][] motorPowers = {
        {0.0, 0.0},
        {0.0, 0.0}
    };

    double[][] leftXMat = {
            { 0.0,  0.0},
            { 0.0,  0.0}
    };

    double[][] leftYMat = {
            { 0.0,  0.0},
            { 0.0,  0.0}
    };

    double[][] rightXMat = {
            { 0.0,  0.0},
            { 0.0,  0.0}
    };

    double[][] rightYMat = {
            { 0.0,  0.0},
            { 0.0,  0.0}
    };

    public MechTeleOp() {
        super();
    }

    @Override
    public void init() {
        setupMotors();
        resetStartTime();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("gpad1LX", gamepad1.left_stick_x);
        telemetry.addData("gpad1LY", gamepad1.left_stick_y);
        driveRobot();
        telemetry.update();
    }

    @Override
    public void stop() {
        moveForward(0.0);
    }

    public void moveForward(double power) {
        motorFL.setPower(Range.clip(calibFL * power, -1, 1));
        motorFR.setPower(Range.clip(calibFR * power, -1, 1));
        motorBL.setPower(Range.clip(calibBL * power, -1, 1));
        motorBR.setPower(Range.clip(calibBR * power, -1, 1));
    }

    public void rotateLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * power);
        motorBL.setPower(calibBL * -power);
        motorBR.setPower(calibBR * power);
    }

    public void straifLeft(double power) {
        motorFL.setPower(calibFL * -power);
        motorFR.setPower(calibFR * -power);
        motorBL.setPower(calibBR * power);
        motorBR.setPower(calibBR * power);
    }

    public void matrixToPowers(double[][] power) {
        motorFL.setPower(calibFL * power[0][0]);
        motorFR.setPower(calibFR * power[0][1]);
        motorBL.setPower(calibBL * power[1][0]);
        motorBR.setPower(calibBR * power[1][1]);
    }

    public double[][] avgPowerMatrix(double[][] leftY, double[][] leftX, double[][] rightY, double[][]rightX){
        double[][] result = {
                { 0.0,  0.0},
                { 0.0,  0.0}
        };

        for (int i = 0; i < 2; i++) {
            for (int k = 0; k < 2; k++) {
                result[i][k] = leftY[i][k] + leftX[i][k] + rightY[i][k] + rightX[i][k];
            }
        }

        for (int i = 0; i < 2; i++) {
            for (int k = 0; k < 2; k++) {
                result[i][k] /= 2;
            }
        }

        return result;
    }

    public void setupMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft"); // frontLeft
        motorFR = hardwareMap.get(DcMotor.class, "frontRight"); // frontRight
        motorBL = hardwareMap.get(DcMotor.class, "backLeft"); // backLeft
        motorBR = hardwareMap.get(DcMotor.class, "backRight"); // backRight

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armOne = hardwareMap.get(DcMotor.class, "armOne");
        armTwo = hardwareMap.get(DcMotor.class, "armTwo");
        armThree = hardwareMap.get(DcMotor.class, "armThree");

        armOne.setDirection(DcMotorSimple.Direction.FORWARD);
        armTwo.setDirection(DcMotorSimple.Direction.FORWARD);
        armThree.setDirection(DcMotorSimple.Direction.FORWARD);

        armOne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armThree.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        leftServo = hardwareMap.get(CRServo.class, "leftServo");
//        rightServo = hardwareMap.get(CRServo.class, "rightServo");
    }

    public void driveRobot() {
        /* I want to change the movement code of the robot to
         * a sumation based system. This means that each movement
         * of the joystick will add some value (positive or negative)
         * to each of the powers of the motors.
         *
         * This would let the robot have an increased range of motion.
         * For instance, if you moved the left stick on a diagonal,
         * the robot currently chooses to either go forward or straif.
         * The addition system will add together some values for each motor
         * to make the robot straif diagonally. */

        /* So far the system, in theory, works exactly like Jimmy's code does
         * This is probably a more visual explanation of what Jimmy is doing
         * with his code. */
        double leftX = getLX(), leftY = getLY(), rightX = getRX(), rightY = getRY();

        if(Math.max(Math.max(Math.abs(leftX), Math.abs(leftY)), Math.max(Math.abs(rightX), Math.abs(rightY))) > 0.1){ // Makes sure that atleast one of the sticks is being pressed
            /* If we stick with a plain averaging system, both joysticks have to be
             * pressed forward or backwards to go max speed. This will also effect
             * the speeds of straifing and turning. A counterbalance can be added
             * to offset that effect.*/

            leftXMat = new double[][]{
                    {-leftX, -leftX}, //Here the negative will probably have to on the bottom motors
                    { leftX,  leftX}
            };

            leftYMat = new double[][] {
                    { leftY,  leftY},
                    { leftY,  leftY}
            };

            rightXMat = new double[][] {
                    {-rightX,  rightX},
                    {-rightX,  rightX}
            };

            rightYMat = new double[][] {
                    { rightY,  rightY},
                    { rightY,  rightY}
            };

            motorPowers = avgPowerMatrix(leftYMat, leftXMat, rightYMat, rightXMat);

            /*New Implementation of old code*/
            // switch (Math.max(abs(leftX), abs(leftY), abs(rightX), abs(rightY)) { // Picks the joystick vector with the most distance from the origin
            //     case leftX:
            //         motorPowers = new double[][]{
            //             {-leftX, -leftX},
            //             { leftX,  leftX}
            //         }
            //         break;                
            //     case leftY:
            //         motorPowers = new double[][]{
            //             { leftY,  leftY},
            //             { leftY,  leftY}
            //         }
            //         break;
            //     case rightX:
            //         motorPowers = new double[][]{
            //             {-rightX,  rightX},
            //             {-rightX,  rightX}
            //         }
            //         break;
            //     case rightY:
            //         motorPowers = new double[][]{
            //             { rightY,  rightY},
            //             { rightY,  rightY}
            //         }
            //         break;
            // }

        }else{
            motorPowers = new double[][] {
                { 0.0,  0.0},
                { 0.0,  0.0}
            };
        }

        matrixToPowers(motorPowers);

        /*Old Code for Joystick Movement*/
        // if (Math.abs(getRX()) < Math.abs(getLX()) || Math.abs(getRX()) < Math.abs(getLY())) {
        //     if (Math.abs(getLX()) < 0.1 && Math.abs(getLY()) < 0.1)
        //         moveForward(0);
        //     else if (Math.abs(getLX()) > Math.abs(getLY()))
        //         straifLeft(Range.clip(getLX(), -1.0, 1.0));
        //     else if (Math.abs(getLY()) > Math.abs(getLX()))
        //         moveForward(Range.clip(getLY(), -1.0, 1.0));
        // } else {
        //     if (Math.abs(getRX()) < 0.1)
        //         moveForward(0);
        //     else
        //         rotateLeft(getRX());
        // }

//        if (gamepad1.dpad_up) {
//            leftServo.setPower(1.0);
//        } else if (gamepad1.dpad_down) {
//            leftServo.setPower(-1.0);
//        } else {
//            leftServo.setPower(0.0);
//        }
//
//        if (gamepad1.y) {
//            rightServo.setPower(1.0);
//        } else if (gamepad1.a) {
//            rightServo.setPower(-1.0);
//        } else {
//            rightServo.setPower(0.0);
//        }

        //arm base (closer to the base of the robot) up and down
        if (gamepad1.dpad_up)
            armOne.setPower(0.5);
        else if (gamepad1.dpad_down)
            armOne.setPower(-0.5);
        else
            armOne.setPower(0.0);

        //arm base (closer to the hand) up and down
        if (gamepad1.dpad_left)
            armTwo.setPower(0.5);
        else if (gamepad1.dpad_right)
            armTwo.setPower(-0.5);
        else
            armTwo.setPower(0.0);

        //hand open close
        if (gamepad1.a)
            armThree.setPower(0.3);
        else if (gamepad1.y)
            armThree.setPower(-0.3);
        else
            armThree.setPower(0.0);


    }

    public float getLX() {
        return gamepad1.left_stick_x;
    }

    public float getLY() {
        return -gamepad1.left_stick_y;
    }

    public float getRX() {
        return -gamepad1.right_stick_x;
    }
    
    public float getRY() { 
        return -gamepad1.right_stick_y; 
    }

}
