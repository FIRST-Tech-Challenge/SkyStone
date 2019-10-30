package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class MecanumAuto_BlueUpper extends MecanumAutoCentral {
    private final double POWER = 0.5;

    public void runOpMode() {

        fL = hardwareMap.dcMotor.get("fL");
        fR = hardwareMap.dcMotor.get("fR");
        bL = hardwareMap.dcMotor.get("bL");
        bR = hardwareMap.dcMotor.get("bR");
        fL.setDirection(DcMotor.Direction.REVERSE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        il = hardwareMap.dcMotor.get("il");
        il.setDirection(DcMotor.Direction.REVERSE);
        il.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ir = hardwareMap.dcMotor.get("ir");
        ir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        int skystone1 = (int) (6 * random());
        int skystone2 = skystone1;

        while (skystone1 == skystone2) {
            skystone2 = (int) (6 * random());
        }
         */

        double gameField = 12 * 12;
        double quarryLength = 48.5;
        double sideWallToQuarry = 47;

        double boxSideLength = 22.75;
        double stoneLength = 8;
        double stoneWidth = 4;
        double stoneBoxHeight = 4;
        double stoneKnobHeight = 1;
        double chassisLength = 16.5;
        double chassisWidth = 18;
        double chassisDiff = abs(chassisWidth - chassisLength);

        waitForStart();

        drive(0.1, -(47.25 - chassisLength));
        strafeLeft(POWER, chassisWidth - 8);
        //Grab please
        drive(0.3, 47.25 - chassisLength);
        //Release please
        strafeLeft(POWER, -(34.5 + (chassisWidth - 8) + 24));
    }
}
