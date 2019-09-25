package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="Prototype Tester", group="none")
//@Disabled
public class PrototypeTester extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;
//    private GyroSensor gyro;
//    private ColorSensor color;
    private Servo armTune; // the servo for the purpose of making the arm turn
    //color sensor arm

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

//        gyro = hardwareMap.gyroSensor.get("gyro");
//        color = hardwareMap.colorSensor.get("color");
        armTune = hardwareMap.servo.get("armTune");

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        armTune.setPosition(0);
        telemetry.addData("armTune", armTune.getPosition());
        telemetry.update();

        boolean prevPos = false;
        boolean pos = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if(Math.abs(gamepad1.left_stick_y) < 0.15){
                setLeftPower(0);
            }
            else {
                setLeftPower(gamepad1.left_stick_y);
            }
            if(Math.abs(gamepad1.right_stick_y) < 0.15){
                setRightPower(0);
            }
            else{
                setRightPower(gamepad1.right_stick_y);
            }

            if (!prevPos && gamepad1.x) {
                if (!pos) {
                    armTune.setPosition(.5);
                    pos = !pos;
                }
                else{
                    armTune.setPosition(0);
                    pos = !pos;
                }
                prevPos = true;
            }
            else if (!gamepad1.x) {
                prevPos = false;
            }
        }
    }
    void setLeftPower(double n){
        left1.setPower(n);
        left2.setPower(n);
    }
    void setRightPower(double n){
        right1.setPower(n);
        right2.setPower(n);
    }
}