package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This class purpose is strictly for the development of an experimental controller that
 * resembles a Proportional controller. The controller that is being developed uses data
 * collected from the encoderTestv3 java file. Further documentation can be found later.
 *
 */

@TeleOp(name = "Redrick Controller", group = "Control")
public class PController extends LinearOpMode {
    public static DcMotor BL;
    ElapsedTime runtime = new ElapsedTime();
    TypexChart chart = new TypexChart();
    private double Kp;

    @Override
    public void runOpMode() throws InterruptedException {
        runtime.reset();

        BL = hardwareMap.get(DcMotor.class, "BL");

        Kp = .75;

        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Encoder Value", BL.getCurrentPosition());
            telemetry.addData("Power", BL.getPower());
            telemetry.addData("Pout", pController(BL, 0));
            telemetry.update();

            double power = pController(BL, 1);

            BL.setPower(power);

            //BL.setPower(gamepad1.left_stick_y);

            /*while (*//*(threshold(BL, 1000, 0)) &&*//* gamepad1.a) {
                BL.setPower(pController(BL, 1000));
                telemetry.update();

            }*/
        }
    }

/*    public boolean threshold(DcMotor wheel, int targetPos, int tolerance) {
        int minLim = targetPos - tolerance;

        return wheel.getCurrentPosition() <= minLim;
    }*/

    public double pController(DcMotor wheel, int targetPos) {
        double Pout_Raw = (Kp) * -((wheel.getCurrentPosition() - targetPos) / targetPos) + 0.05;
/*        Double Pout_Clipped = Range.clip(Pout_Raw, -1, 1);

        return Pout_Clipped;*/
        return Pout_Raw;
    }

/*    public double distanceVector(DcMotor wheel, int targetPos) {
        int distanceNeeded = targetPos - wheel.getCurrentPosition();
        if (distanceNeeded < 0.1) {
            return 0.1;
        } else if (distanceNeeded > 0.1) {
            return -0.1;
        } else {
            return 0.1;
        }

    }*/

}
