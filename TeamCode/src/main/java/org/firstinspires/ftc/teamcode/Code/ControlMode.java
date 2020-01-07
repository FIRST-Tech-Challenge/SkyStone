// Jun Park
// Last Update: 11/18/2019

package org.firstinspires.ftc.teamcode.Code;

// Import
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//This class will include all of the action functions such as Movement
@TeleOp(name = "Control Mode", group = "Jun")
public class ControlMode extends OpMode {
    //, Set the motor variable name that the program will use.
    DcMotor motorFL, motorFR; // Front Left / Right motor
    DcMotor motorBL, motorBR; // Back Left / Right motor
    DcMotor motorArmVertical, motorArmHorizontal, motorArmGrab; // Arm movement motor

    public ControlMode() {
        super();
    }

    @Override
    public void init() { // This code will executes when player pressed the button "init" on the phone.
        // Use the same motor name when you sets on configuration.
        motorFL = hardwareMap.dcMotor.get("FL"); // Front Left
        motorFR = hardwareMap.dcMotor.get("FR"); // Front Right
        motorBL = hardwareMap.dcMotor.get("BL"); // Back Left
        motorBR = hardwareMap.dcMotor.get("BR"); // Back Right

        motorArmVertical = hardwareMap.dcMotor.get("AV"); // Arm Vertical Movement
        motorArmHorizontal = hardwareMap.dcMotor.get("AH"); // Arm Horizontal Movement
        motorArmGrab = hardwareMap.dcMotor.get("AG"); // Arm Grab

        // This code is used set the directions of the motors.
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorArmVertical.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArmHorizontal.setDirection(DcMotorSimple.Direction.FORWARD);
        motorArmGrab.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() { // This code will executes over and over after the start button is pressed.
        controlMovement(); // Control the Robot based on the received data form the controller phone.
        reportData(); // Report the data to the controller phone.
    }

    boolean grab = false;
    public void controlMovement() {
        //left joystick of the first controller will control the movement of the robot.
        if (gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0) { // If left stick is moving

            // Determine which Vertical value or Horizontal value has a bigger value
            if (Math.abs(gamepad1.left_stick_x) > Math.abs(gamepad1.left_stick_y)) // If Vertical value is greater than Horizontal value
                rotate((double) gamepad1.left_stick_x * 0.5); // Rotate the robot to that direction.
            else if (Math.abs(gamepad1.left_stick_x) < Math.abs(gamepad1.left_stick_y)) // If Horizontal value is greater than Vertical value
                moveForward((double) gamepad1.left_stick_y); // Move forward or backward.
        } else {
            moveForward(0.0);
        }

        // Arm Vertical movement.
        if (gamepad1.dpad_up)  // Move up when dpad up pressed.
            motorArmVertical.setPower(0.5);
        else if (gamepad1.dpad_down) // Move down when dpad up pressed.
            motorArmVertical.setPower(-0.5);
        else // Stop when dpad is not pressed anymore.
            motorArmVertical.setPower(0);

        // Arm Horizontal movement.
        if (gamepad1.dpad_up)  // Move left when dpad left pressed.
            motorArmHorizontal.setPower(0.5);
        else if (gamepad1.dpad_down) // Move right when dpad right pressed.
            motorArmHorizontal.setPower(-0.5);
        else // Stop when dpad is not pressed anymore.
            motorArmHorizontal.setPower(0);

        // Arm Grab movement.
        if (gamepad1.a)  // Grab the block when 'a' button pressed.
            motorArmGrab.setPower(0.5);
        else if (gamepad1.b) // Release the block when'b' button pressed.
            motorArmGrab.setPower(-0.5);
        else // Stop when button is not pressed anymore.
            motorArmGrab.setPower(0);
    }

    //helper method to report data to controller phone.
    public void reportData() {
        //report Gamepad data
        telemetry.addData("Gamepad leftX", gamepad1.left_stick_x);
        telemetry.addData("Gamepad leftY", gamepad1.left_stick_y);
        telemetry.addData("Gamepad dpad up/down", motorArmVertical.getPower());
        telemetry.addData("Gamepad dpad left/right", motorArmHorizontal.getPower());
        telemetry.addData("Gamepad a (Grab)", gamepad1.a);
        telemetry.addData("Gamepad b (Release)", gamepad1.b);

        //report motors' power
        telemetry.addData("motorFL", motorFL.getPower());
        telemetry.addData("motorFR", motorFR.getPower());
        telemetry.addData("motorBL", motorBL.getPower());
        telemetry.addData("motorBR", motorBR.getPower());
        telemetry.addData("motorArmVertical", motorArmVertical.getPower());
        telemetry.addData("motorArmHorizontal", motorArmHorizontal.getPower());
        telemetry.addData("motorArmGrab", motorArmGrab.getPower());

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