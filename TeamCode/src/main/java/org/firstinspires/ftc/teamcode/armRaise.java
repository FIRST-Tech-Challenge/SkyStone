package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "armTester", group = "Tester")
public class armRaise extends LinearOpMode {
    public DcMotor armRaise;
    public DcMotor armExtend;
    public int TPR = 1120;
    public int gearRatio = 1;
    public double circumference = 4 * 3.1459;
    public double unitRate = TPR / circumference;
    public DcMotor BL;
    public DcMotor TR;
    ElapsedTime time = new ElapsedTime();

    public int boolPower(boolean power) {
        if (power == true) {
            return (1);
        }
        else {
            return (0);
        }
    }

    public boolean toleranceStoppage(DcMotor wheel, int targetPos, double tolerance) {
        int intTolerance = (int) Math.round(tolerance);
        return wheel.getCurrentPosition() <= targetPos-intTolerance;
    }

    public int targetPos(double dist) {
        int position = (int) Math.round(unitRate * dist);
        if (0 == 0) {
            return position;
        } else {
            return 0;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
    armRaise = hardwareMap.get(DcMotor.class, "armRaiser");
    armExtend = hardwareMap.get(DcMotor.class, "armExtend");

    //armRaise.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Encoder Pos", armRaise.getCurrentPosition());

            while ((toleranceStoppage(BL, targetPos(6600), 26.7)) && gamepad1.a) {
                BL.setPower(1);
                telemetry.update();
            }


            telemetry.update();

        }
    }
}
