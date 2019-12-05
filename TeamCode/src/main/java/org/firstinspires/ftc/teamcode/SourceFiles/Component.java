package org.firstinspires.ftc.teamcode.SourceFiles;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Component {
    private HardwareMap hardwareMap;

    public DcMotor leftIntake;
    public DcMotor rightIntake;

    public Servo leftLatch;
    public Servo rightLatch;

    // TODO: Add claw mechanism

    public ColorSensor colorSensor;

    public boolean isLatched = false;
    public String latchStatus = "Pending";

    public Component(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        leftIntake = hardwareMap.get(DcMotor.class, "left intake");
        rightIntake = hardwareMap.get(DcMotor.class, "right intake");

        leftLatch = hardwareMap.get(Servo.class, "left latch");

        rightLatch = hardwareMap.get(Servo.class, "right latch");
    }

    public void latch(String mode) {
        if (mode.toUpperCase().equals("LATCH")) {
            leftLatch.setPosition(0.5);
            rightLatch.setPosition(0.3);
        } else if (mode.toUpperCase().equals("UNLATCH")) {
            leftLatch.setPosition(1);
            rightLatch.setPosition(0);
        }
    }

    public void intake(String mode) {
        if (mode.toUpperCase().equals("INTAKE")) {
            leftIntake.setPower(0.5);
            rightIntake.setPower(-0.5);
        } else if (mode.toUpperCase().equals("RELEASE")) {
            leftIntake.setPower(-0.2);
            rightIntake.setPower(0.2);
        } else if (mode.toUpperCase().equals("STOP")){
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    //TODO: add color sensor code
}
