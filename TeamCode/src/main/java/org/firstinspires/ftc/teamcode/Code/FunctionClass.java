//Jun Park
package org.firstinspires.ftc.teamcode.Code;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//This class will include all of the action functions such as Movement
@TeleOp(name = "Function Test", group = "Iterative Opmode")
public class FunctionClass extends OpMode {
    DcMotor motorFL, motorFR; //, motorBL, motorBR;

    public FunctionClass() {
        super();
    }

    @Override
    public void init() { // This code will executes when player pressed the button "init" on the phone.
        // Use the same motor name when you sets on configuration.
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        //motorBL = hardwareMap.dcMotor.get("motorBL");
        //motorBR = hardwareMap.dcMotor.get("motorBR");

        // This code is used set the directions of the motors.
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() { // This code will executes over and over after the start button is pressed.
        controlMovement(); // Control the Robot based on the received data form the controller phone.
        reportData(); // Report the data to the controller phone.
    }

    public void controlMovement() {
        //left joystick of the first controller will control the movement of the robot.
        if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) {
            if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y))
                rotate((double) -gamepad1.left_stick_x);
            else if (Math.abs(gamepad1.left_stick_x) < Math.abs(gamepad1.left_stick_y))
                moveForward((double) -gamepad1.left_stick_y);
        } else {
            double leftPower = -gamepad1.right_stick_y;
            double rightPower = -gamepad1.right_stick_y;

            leftPower += (gamepad1.right_stick_x);
            rightPower -= (gamepad1.right_stick_x);

            motorFL.setPower(leftPower);
            motorFR.setPower(rightPower);
            //motorBL.setPower(leftPower);
            //motorBR.setPower(rightPower);
        }
    }

    //helper method to report data to controller phone.
    public void reportData() {
        //report gamepad1 left joystick data
        telemetry.addData("gamepad1 leftX", gamepad1.left_stick_x);
        telemetry.addData("gamepad1 leftY", gamepad1.left_stick_y);

        //report motors' power
        telemetry.addData("motorFL", motorFL.getPower());
        telemetry.addData("motorFR", motorFR.getPower());
        //telemetry.addData("motorBL", motorBL.getPower());
        //telemetry.addData("motorBR", motorBR.getPower());

        //update the data screen.
        telemetry.update();
    }

    //helper method to reset all motors to not moving.
    public void resetMotors() {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        //motorBL.setPower(0.0);
        //motorBR.setPower(0.0);
    }

    //helper method to move the robot
    //pass in negative value to move backward
    public void moveForward(double power) { // Move backward by giving negative value to the valuable "power"
        motorFL.setPower(power);
        motorFR.setPower(power);
        //motorBL.setPower(power);
        //motorBR.setPower(power);
    }

    //helper method to rotate the robot
    //pass in negative value to rotate right
    public void rotate(double power) { // Rotating method, Negative value to rotate right side.
        motorFL.setPower(-power);
        motorFR.setPower(power);
        //motorBL.setPower(-power);
        //motorBR.setPower(power);
    }
}