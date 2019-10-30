//Jun Park
package org.firstinspires.ftc.teamcode.Code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

//This class will include all of the action functions such as Movement
@TeleOp(name = "Control Mode", group = "Jun")
public class ControlMode extends OpMode {
    DcMotor motorFL, motorFR, motorBL, motorBR; //, motorArm1, motorArm2, motorArm3;

    public ControlMode() {
        super();
    }

    @Override
    public void init() { // This code will executes when player pressed the button "init" on the phone.
        // Use the same motor name when you sets on configuration.
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");

//        motorArm1 = hardwareMap.dcMotor.get("A1");
//        motorArm2 = hardwareMap.dcMotor.get("A2");
//        motorArm3 = hardwareMap.dcMotor.get("A3");

        // This code is used set the directions of the motors.
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

//        motorArm1.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorArm2.setDirection(DcMotorSimple.Direction.FORWARD);
//        motorArm3.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() { // This code will executes over and over after the start button is pressed.
        controlMovement(); // Control the Robot based on the received data form the controller phone.
        reportData(); // Report the data to the controller phone.
    }

    boolean grab = false;
    public void controlMovement() {
        //left joystick of the first controller will control the movement of the robot.
        if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
            if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y))
                rotate((double) gamepad1.left_stick_x * 0.5);
            else if (Math.abs(gamepad1.left_stick_x) < Math.abs(gamepad1.left_stick_y))
                moveForward((double) gamepad1.left_stick_y);
        } else {
            moveForward(0.0);
        }


        /*else if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0 ) {
            double power = Math.abs(gamepad1.right_stick_x);
            if (Math.abs(gamepad1.right_stick_x) > Math.abs(gamepad1.right_stick_y)) {
                if (gamepad1.right_stick_x < 0) { // Wheels rotate outside to move the vehicle left vertically
                    motorFL.setPower(power);
                    motorFR.setPower(power);
                    motorBL.setPower(-power);
                    motorBR.setPower(-power);
                } else { // Wheels rotate inside to move the vehicle right vertically
                    motorFL.setPower(-power);
                    motorFR.setPower(-power);
                    motorBL.setPower(power);
                    motorBR.setPower(power);
                }
            }
        } else if (Math.abs(gamepad1.left_stick_x) < Math.abs(gamepad1.left_stick_y)) {
            moveForward((double) -gamepad1.left_stick_y);
        } else {
            double leftPower = -gamepad1.right_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            leftPower += (gamepad1.right_stick_x);
            rightPower -= (gamepad1.right_stick_x);

            motorFL.setPower(leftPower);
            motorFR.setPower(rightPower);
            motorBL.setPower(leftPower);
            motorBR.setPower(rightPower);
        } */

//        if (gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0)
//            moveForward((0));

//        if (gamepad1.dpad_up)
//            motorArm1.setPower(0.5);
//        else if (gamepad1.dpad_down)
//            motorArm1.setPower(-0.5);
//        else
//            motorArm1.setPower(0);
//
//        if (gamepad1.dpad_left)
//            motorArm3.setPower(0.5);
//        else if (gamepad1.dpad_right)
//            motorArm3.setPower(-0.5);
//        else
//            motorArm3.setPower(0);
//
//        if (gamepad1.b)
//            motorArm2.setPower(0.25);
//        else if (gamepad1.a)
//            motorArm2.setPower(-0.25);
//        else
//            motorArm2.setPower(0.0);
    }

    //helper method to report data to controller phone.
    public void reportData() {
        //report gamepad1 left joystick data
        telemetry.addData("gamepad1 leftX", gamepad1.left_stick_x);
        telemetry.addData("gamepad1 leftY", gamepad1.left_stick_y);
        telemetry.addData("gamepad1 dpad a", gamepad1.a);
        telemetry.addData("gamepad1 dpad b", gamepad1.b);

        //report motors' power
        telemetry.addData("motorFL", motorFL.getPower());
        telemetry.addData("motorFR", motorFR.getPower());
        telemetry.addData("motorBL", motorBL.getPower());
        telemetry.addData("motorBR", motorBR.getPower());

//        telemetry.addData("A1", motorArm1.getPower());
//        telemetry.addData("A2", motorArm2.getPower());
//        telemetry.addData("A3", motorArm3.getPower());
        //update the data screen.
        telemetry.update();
    }

    public void moveForward(double power) { // Move backward by giving negative value to the valuable "power"
        motorFL.setPower(power);
        motorFR.setPower(power);
        motorBL.setPower(power);
        motorBR.setPower(power);
    }

    public void rotate(double power) { // Rotating method, Negative value to rotate right side.
        motorFL.setPower(-power);
        motorFR.setPower(power);
        motorBL.setPower(-power);
        motorBR.setPower(power);
    }
}