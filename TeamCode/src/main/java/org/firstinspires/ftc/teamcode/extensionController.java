package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "extensionTester", group = "REDWOOD")
public class extensionController extends LinearOpMode {

    public DcMotor armExtender;

    @Override
    public void runOpMode() throws InterruptedException {
        armExtender = hardwareMap.get(DcMotor.class, "armExtender");

        armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while(opModeIsActive()) {
            whileLoopController(700);

        }
    }

    public void whileLoopController(int encoder) {
        while (((armExtender.getCurrentPosition()<(encoder-10)) ||  (armExtender.getCurrentPosition()>(encoder+10))) && gamepad1.left_trigger>0) {
            armExtender.setPower(0.4);
        }
    }

    public void extController(double distance) {
        while ( ((armExtender.getCurrentPosition()<(distance-10)) ||  (armExtender.getCurrentPosition()>(distance+10))) && gamepad1.left_trigger>0) {
            double newPos = (double)armExtender.getCurrentPosition();
            double power = ((newPos - distance)/distance);
            double clippedPower = -1*Range.clip(power, -0.4, 0.4);
            armExtender.setPower(-power);
            telemetry.addData("Looping?", ((armExtender.getCurrentPosition()<(distance-10)) ||  (armExtender.getCurrentPosition()>(distance+10))) && gamepad1.left_trigger>0);
            telemetry.addData("Power Output", clippedPower + "-" + power);
            telemetry.update();
        }
    }

    public double powerOutput(double distance) {
        double newPos = (double)armExtender.getCurrentPosition();
        double power = ((newPos - distance)/distance);
        return power;
    }

    /*public void extensionController(int targDist) {
        if (armExtender.getCurrentPosition()<targDist) {
            telemetry.addData("Position1", armExtender.getCurrentPosition());
            armExtender.setPower(-(Range.clip(((armExtender.getCurrentPosition()-targDist)/targDist), -0.4, 0.4)));
            telemetry.update();
        }
        else if (armExtender.getCurrentPosition()>targDist) {
            armExtender.setPower(Range.clip(-((armExtender.getCurrentPosition()-targDist)/targDist), -0.4, 0.4));
            telemetry.addData("Position2", armExtender.getCurrentPosition());
            telemetry.update();
        }
    }*/
}
