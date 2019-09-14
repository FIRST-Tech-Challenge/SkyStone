package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This encoder test program serves as a form of data collection for the tuning
 * of the P-Controller that is under creation at the moment. Please do not tamper with
 * the values of this program unless given permission to.
 *
 */


@Autonomous(name = "encoderTestv3", group = "Experimental")
public class BasicAuto extends LinearOpMode {

    public int TPR = 1120;
    public int gearRatio = 1;
    public double circumference = 4 * 3.1459;
    public double unitRate = TPR / circumference;
    public DcMotor BL;
    public DcMotor TR;
    TypexChart chart = new TypexChart();
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        BL = hardwareMap.get(DcMotor.class, "BL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        chart.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("Encoder", BL.getCurrentPosition());
            telemetry.addData("Past", threshold(BL, targetPos(10)));

            TR.setPower(-gamepad1.right_stick_y);

            BL.setPower(gamepad1.left_stick_y);

            while ((threshold(BL, targetPos(10))) && gamepad1.a) {
                BL.setPower(0.1);
            }

            telemetry.update();
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
        return wheel.getCurrentPosition() <= targetPos;
    }
}
