// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Autonomous(name="Drive Encoder2", group="Exercises")

public class HardwareBot
{
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
    public DcMotor heavyDutyArm = null;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public HardwareBot() {
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        heavyDutyArm = hwMap.get(DcMotor.class, "arm");

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

        heavyDutyArm.setPower(0);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        heavyDutyArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
