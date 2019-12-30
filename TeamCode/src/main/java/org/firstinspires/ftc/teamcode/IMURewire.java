package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMURewire")
public class IMURewire extends LinearOpMode {
    TypexChart chart = new TypexChart();
    ElapsedTime runtime = new ElapsedTime();
    CONSTANTS constants = new CONSTANTS();

    double correction;

    Orientation lastAngle;

    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        chart.init(hardwareMap);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;


        imu.initialize(parameters);

        while(opModeIsActive()&&!imu.isGyroCalibrated()){

        }

        telemetry.addData("Done", "");
        telemetry.update();

        resetAngle();


        waitForStart();
        while(opModeIsActive()){
            correction = correction(0);

            if (correction<0){
                chart.TL.setPower(constants.HALFPOWER - correction);
                chart.BL.setPower(constants.HALFPOWER - correction);
                chart.TR.setPower(constants.HALFPOWER + correction);
                chart.BR.setPower(constants.HALFPOWER + correction);
            }
            else if (correction>0){
                chart.TL.setPower(constants.HALFPOWER + correction);
                chart.BL.setPower(constants.HALFPOWER + correction);
                chart.TR.setPower(constants.HALFPOWER - correction);
                chart.BR.setPower(constants.HALFPOWER - correction);
            }
        }
    }

    public void resetAngle() {
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double correction(double targetAngle){
        double gain = 0.1;
        double correction = (double)(lastAngle.firstAngle - targetAngle)/(double)targetAngle;

        return correction * gain;
    }
}
