package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "TestAutonomous")
public class TestAutonomous extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    final double calibFL = 1.00f;
    final double calibFR = 1.00f;
    final double calibBL = 1.00f;
    final double calibBR = 1.00f;

    @Override
    public void runOpMode() {
        initMotors();
        telemetry.addData("status", "Initialized");
        waitForStart();

        doAction("forward", 1.0);

        sleep(1000);

        stop();
    }

    public void initMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "frontLeft"); // frontLeft
        motorFR = hardwareMap.get(DcMotor.class, "frontRight"); // frontRight
        motorBL = hardwareMap.get(DcMotor.class, "backLeft"); // backLeft
        motorBR = hardwareMap.get(DcMotor.class, "backRight"); // backRight

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void doAction(String action, double degree) {
        ActionClass.doAction(calibFL, calibFR, calibBL, calibBR, motorFL, motorFR, motorBL, motorBR, action, degree);
    }

}

