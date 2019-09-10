package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class encoderTest extends LinearOpMode {

    public static DcMotor test1;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        test1 = hardwareMap.get(DcMotor.class, "test1");
        test1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            telemetry.addData("Time:", runtime.seconds());
            telemetry.addData("Encoder Position:", test1.getCurrentPosition() );
            telemetry.update();

            test1.setPower(gamepad1.right_stick_y);
            while (gamepad1.left_bumper) {
                runToPos(5, 0);
            }
        }
    }

    public void runToPos(int tolerance, int targetPos) {
        while(switchCase(tolerance, targetPos)) {
            test1.setTargetPosition(targetPos);
            test1.setPower(0.25);
        }
    }


    public boolean switchCase(int tolerance, int targetPos) {
        int minLim = targetPos - tolerance;
        int maxLim = targetPos + tolerance;

        if ( (test1.getCurrentPosition() < minLim) && (test1.getCurrentPosition() > maxLim) ) {
            return true;
        }
        else if ( (test1.getCurrentPosition() > minLim) && (test1.getCurrentPosition() < maxLim) ) {
            return false;
        }
        else {
            return false;
        }
    }
}
