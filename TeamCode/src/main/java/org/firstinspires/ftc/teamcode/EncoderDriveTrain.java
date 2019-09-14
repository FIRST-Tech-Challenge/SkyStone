package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "EncoderAuto", group = "Experimental")
public class EncoderDriveTrain extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        TypexChart chart = new TypexChart();

        chart.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Runtime", runtime.seconds());
            telemetry.addData("Encoder TL", chart.TL.getCurrentPosition());
            telemetry.update();

            chart.TL.setPower(-gamepad1.left_stick_y);
            chart.BL.setPower(-gamepad1.left_stick_y);

            chart.TR.setPower(-gamepad1.right_stick_y);
            chart.BR.setPower(-gamepad1.right_stick_y);

        }
    }
}
