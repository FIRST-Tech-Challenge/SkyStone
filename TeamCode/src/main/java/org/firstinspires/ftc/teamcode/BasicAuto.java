package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BasicAuto", group = "Basic")
public class BasicAuto extends LinearOpMode {

    TypexChart chart = new TypexChart();
    ElapsedTime runtime = new ElapsedTime();

    public double[] voltageRange = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    public double[] controlValve = {27};

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        while(opModeIsActive()) {

        }
    }

    public void straightDriveTime(double power) {
        //runtime.reset();
        chart.TR.setPower(power);
        chart.TL.setPower(power);
        chart.BR.setPower(power);
        chart.BL.setPower(power);
    }

    public boolean eachWheelThreshold(DcMotor wheel, int targetPos, int tolerance) {
        
        return wheel.getCurrentPosition() <=
    }

    public boolean rWeThereYet() {

    }

}
