package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Math.abs;
import static java.lang.Math.random;

@Autonomous

public class MecanumAuto_Stupid extends MecanumAutoCentral {
    private final double POWER = 0.5;

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

        strafeLeft(POWER, 10);
        drive(POWER, 48 - chassisLength);
        //Grab please
        drive(POWER, -(46 - chassisLength));
        //Release please
        strafeLeft(POWER, -28);

    }
}
