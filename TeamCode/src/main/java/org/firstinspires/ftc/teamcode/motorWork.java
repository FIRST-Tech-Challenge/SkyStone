package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "please")
public class motorWork extends LinearOpMode {

    public DcMotor armE;
    public DcMotor armR;
    public double unitRate = 75.59;


    @Override
    public void runOpMode() throws InterruptedException {

        armE = hardwareMap.get(DcMotor.class, "armExtender");
        armR = hardwareMap.get(DcMotor.class, "armRaiser");

        armE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()) {
            while ((threshold(armE, 288)) && gamepad1.left_trigger>0) {
                telemetry.addData("Iteration:", armE.getCurrentPosition());
                armE.setPower(0.4);
                telemetry.update();
            }
            armE.setPower(0);
        }
    }
    public int targetPos(double dist) {
        int position = (int) Math.round(unitRate * dist);
        if (0 == 0) {
            return position;
        } else {
            return 0;
        }
    }

    public boolean threshold(DcMotor wheel, int targetPos) {
        int tolerance = targetPos - 5;
        return wheel.getCurrentPosition() <= tolerance;
    }
}
