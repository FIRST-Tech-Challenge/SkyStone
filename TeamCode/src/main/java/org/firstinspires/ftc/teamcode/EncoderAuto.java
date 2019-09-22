package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Encoder Auto", group="Linear Opmode")
public class EncoderAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left1;
    private DcMotor left2;
    private DcMotor right1;
    private DcMotor right2;
    private Servo armTune;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        left1 = hardwareMap.dcMotor.get("left1");
        left2 = hardwareMap.dcMotor.get("left2");
        right1 = hardwareMap.dcMotor.get("right1");
        right2 = hardwareMap.dcMotor.get("right2");

        armTune = hardwareMap.servo.get("armTune");

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        left2.setPower(1);
        left1.setPower(1);
        right2.setPower(1);
        right1.setPower(1);

        while(left1.getCurrentPosition() < 500) {

        }

        left2.setPower(0);
        left1.setPower(0);
        right2.setPower(0);
        right1.setPower(0);
    }
}
