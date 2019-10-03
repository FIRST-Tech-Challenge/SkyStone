//Jun Park
package org.firstinspires.ftc.teamcode.Code;

import java.lang.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* DO NOT test this mode without fixing the vehicle.
 * It ignore the user inputs from the gamepad, which means player can not control it.
 */

//This class is for the matching the motor name with the coded name and configuration name.
@TeleOp(name = "Motor Test (Warning)", group = "Jun")
public class MotorNameTest extends OpMode {
    DcMotor motorFL, motorFR, motorBL, motorBR;

    public MotorNameTest() {
        super();
    }

    @Override
    public void init() { // This code will executes when player pressed the button "init" on the phone.
        // Use the same motor name when you sets on configuration.
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");

        // This code is used set the directions of the motors.
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() { // This code will executes over and over after the start button is pressed.
        try {
            Thread.sleep(1500);
            motorFL.setPower(0.5);
            reportData();
            telemetry.update();

            Thread.sleep(1500);
            motorFL.setPower(0);
            motorFR.setPower(0.5);
            reportData();
            telemetry.update();

            Thread.sleep(1500);
            motorFR.setPower(0);
            motorBL.setPower(0.5);
            reportData();
            telemetry.update();

            Thread.sleep(1500);
            motorBL.setPower(0);
            motorBR.setPower(0.5);
            reportData();
            telemetry.update();

            Thread.sleep(1500);
            motorBR.setPower(0);
            motorFL.setPower(0.5);
            motorFR.setPower(0.5);
            motorBL.setPower(0.5);
            motorBR.setPower(0.5);
            reportData();
            telemetry.update();

            Thread.sleep(1500);
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        } catch (InterruptedException e) {
            System.out.println("got interrupted!");
        }
        reportData();
        telemetry.update();
    }

    //helper method to report data to controller phone.
    public void reportData() {
        //report motors' power
        telemetry.addData("motorFL", motorFL.getPower());
        telemetry.addData("motorFR", motorFR.getPower());
        telemetry.addData("motorBL", motorBL.getPower());
        telemetry.addData("motorBR", motorBR.getPower());
        telemetry.update();
    }
}