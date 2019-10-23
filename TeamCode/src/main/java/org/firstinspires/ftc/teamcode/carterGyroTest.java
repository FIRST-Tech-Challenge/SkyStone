package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

@TeleOp(name = "REDWOOD", group = "REDWOOD")
public class carterGyroTest extends OpMode {
    ElapsedTime runtime = new ElapsedTime();

    public DcMotor armRaiser;
    public DcMotor armExtended;
    GyroSensor gyro;

    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    public Servo grab;
    public Servo grabRot;
    @Override
    public void init() {
        TL = hardwareMap.get(DcMotor.class, "TL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        armRaiser = hardwareMap.get(DcMotor.class, "armRaiser");
        armExtended = hardwareMap.get(DcMotor.class, "armExtender");

        grab = hardwareMap.get(Servo.class, "grab");
        grabRot = hardwareMap.get(Servo.class, "grabRot");


        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro=hardwareMap.get(GyroSensor.class,"gyro");
        gyro.resetZAxisIntegrator();

        runtime.reset();
        gyro.calibrate();

        grabRot.setPosition(0.3);

    }


    int armLevel = 0;
    double[] armAngles = {0,};
    double[] armDistance = {16, 22, };
    int rawz = 0;

    double grabPos = 0;
    double grabRotPos = 0;

    public boolean armLevelUp = false;
    public boolean armLevelDown = false;

    @Override
    public void loop() {
        TL = hardwareMap.get(DcMotor.class, "TL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        armRaiser = hardwareMap.get(DcMotor.class, "armRaiser");
        armExtended = hardwareMap.get(DcMotor.class, "armExtender");

        TL.setPower(gamepad1.left_stick_y);
        BL.setPower(gamepad1.left_stick_y);
        TR.setPower(-gamepad1.right_stick_y);
        BR.setPower(-gamepad1.right_stick_y);

        //Arm Raise and Extend Manual Control
        armRaiser.setPower(-gamepad2.right_stick_y);
        armExtended.setPower(-gamepad2.left_stick_y);

        //Building Level Control
        armLevel();
        armLevelUp = false;
        armLevelDown = false;

        //Grabber Rotation Function
        grabberRotation();

        //Grabber Claw actual grabbing Action
        if (gamepad1.x) {
            grabPos = grabPos + 0.01;
        }
        if (gamepad1.y) {
            grabPos = grabPos - 0.01;
        }
        grab.setPosition(grabPos);


        telemetry.addData("Servo1", grabRot.getPosition());
        telemetry.addData("Servo2", grab.getPosition());

        if (gamepad1.b) {
            rawz = gyro.getHeading();
        }
        telemetry.addData("Head", rawz);
        telemetry.addData("Encoder Ext", armExtended.getCurrentPosition());
        telemetry.addData("Encoder Raise", armRaiser.getCurrentPosition());
        telemetry.addData("Arm Level:", armLevel + 1);
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
    }

    public void armLevel() {
        if (gamepad1.dpad_up) {
            armLevelUp = true;
        }
        if (armLevelUp) {
            armLevel++;
        }
        if (gamepad1.dpad_down) {
            armLevelDown = true;
        }
        if (armLevelDown) {
            armLevel--;
        }
    }

    public void grabberRotation() {
        if (gamepad1.left_bumper) {
            grabRot.setPosition(0.3);
        }
        if (gamepad1.right_bumper) {
            grabRot.setPosition(0.8);
        }
    }

/*    public int getGyroHeading(int[] a) {
        int count = 1, tempCount;
        int common = a[0];
        int temp = 0;
        for (int i = 0; i < (a.length - 1); i++) {
            temp = a[i];
            tempCount = 0;
            for (int j = 1; j < a.length; j++) {
                if (temp == a[j]) {
                    tempCount++;
                }
                if (tempCount > count) {
                    common = temp;
                    count = tempCount;
                }
            }
        }
        return common;
    }*/
}

