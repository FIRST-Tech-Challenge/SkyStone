package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Prototype Tester", group="Linear Opmode")
//@Disabled
public class PrototypeTester extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;
    private GyroSensor gyro;
    private ColorSensor color;
    private Servo armTune;
    //color sensor arm

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        gyro = hardwareMap.gyroSensor.get("gyro");
        color = hardwareMap.colorSensor.get("color");
        armTune = hardwareMap.servo.get("servo0");

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        armTune.setPosition(0.5);

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
            telemetry.addData("armTune", armTune.getPosition());//port
            telemetry.update();

            if (gamepad1.x) {
               armTune.setPosition(.75);
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