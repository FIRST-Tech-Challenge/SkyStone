package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Encoder Value Tester", group = "REDWOOD")
public class encoderValueReader extends LinearOpMode {
    public DcMotor armRaiser;
    public DcMotor armExtender;

    public double powerOutput(double distance) {
        double newPos = (double)armExtender.getCurrentPosition();
        double power = ((newPos - distance)/distance);
        return power;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();

        armRaiser = hardwareMap.get(DcMotor.class, "armRaiser");
        armExtender = hardwareMap.get(DcMotor.class, "armExtender");
        armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Encoder Value for Arm Extender", armExtender.getCurrentPosition());
            telemetry.addData("Encoder Value for Arm Raiser", armRaiser.getCurrentPosition());

            armRaiser.setPower(gamepad1.left_stick_y);
            //armExtender.setPower(powerOutput(700.00));
            armExtender.setPower(gamepad1.right_stick_y);

            telemetry.update();
        }
    }
}
