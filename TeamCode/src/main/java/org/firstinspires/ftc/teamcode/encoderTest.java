package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "EncoderTestv2", group = "basic")
public class encoderTest extends LinearOpMode {

    public static DcMotor test1;
    public int position;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        TypexChart chart = new TypexChart();

        test1 = hardwareMap.get(DcMotor.class, "BL");
        test1.setTargetPosition(0);
        test1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            telemetry.addData("Time:", runtime.seconds());
            telemetry.addData("Encoder Position:", test1.getCurrentPosition() );
            telemetry.update();

            test1.setPower(gamepad1.right_stick_y);
        }
    }

    public void runToPos(int tolerance, int targetPos) {
        while(switchCase(tolerance, targetPos)) {
            test1.setTargetPosition(targetPos);
            test1.setPower(1);
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
