package org.firstinspires.ftc.teamcode;

import static java.lang.Math.random;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class Octo358MecanumAuto_PlaygroundTest extends Octo358MecanumAutoPlayground {

    private final double POWER = 0.6;

    public void runOpMode() {

        fL = hardwareMap.dcMotor.get("0");
        fR = hardwareMap.dcMotor.get("2");
        bL = hardwareMap.dcMotor.get("1");
        bR = hardwareMap.dcMotor.get("3");
        fL.setDirection(DcMotor.Direction.REVERSE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int skystone1 = (int) (6 * random());
        int skystone2 = skystone1;

        while (skystone1 == skystone2) {
            skystone2 = (int) (6 * random());
        }

        waitForStart();

        drive(POWER, 120);

        strafeLeft(POWER, 20);

        rotate(POWER, 360 * 4);

    }

}
