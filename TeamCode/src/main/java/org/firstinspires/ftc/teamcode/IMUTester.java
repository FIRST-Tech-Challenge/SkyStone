package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMUTEster", group = "IMU")
@Disabled
public class IMUTester extends LinearOpMode {

    public BNO055IMU imu;
    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    Orientation angle;
    Orientation lastAngles;

    public double power = 0.3;
    public double globalAngle = 0;
    public double correction;

    @Override
    public void runOpMode() throws InterruptedException {

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        TL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = false;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu.initialize(parameters);

        telemetry.addData("Calibrating the imu ", "...");

        while(!imu.isGyroCalibrated() && opModeIsActive()){
            sleep(50);
            telemetry.update();
        }

        telemetry.addData("Calibration Status: ", "Complete");
        telemetry.update();

        //resetAngle();

        waitForStart();
        while (opModeIsActive()){
            correction = checkDirection();

            //telemetry.addData("getAngle(): ", getAngle());
            while(gamepad1.a){
              //walkStraight(0);
                TL.setPower(power - correction);
                BL.setPower(power - correction);
                TR.setPower(power + correction);
                BR.setPower(power + correction);
            }
            TL.setPower(0);
            BL.setPower(0);
            TR.setPower(0);
            BR.setPower(0);
            telemetry.update();
        }
    }
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double dAngles = angles.firstAngle - lastAngles.firstAngle;

        if (dAngles<-180) dAngles+=360;
        else if (dAngles>180) dAngles-=360;

        globalAngle += dAngles;

        lastAngles = angles;

        return globalAngle;
    }

    public double checkDirection() {
        double correction, gain = 0.10, angle;
        angle = getAngle();

        if (angle ==0){
            correction = 0;
        }
        else correction = -angle;

        correction = correction * gain;
        return
                correction;
    }

    public void walkStraight(double targAngle) {
        double power = 0.2;
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double delta = angle.firstAngle - targAngle;
        telemetry.addData("Here is the delta Angle: ", delta);
        telemetry.addData("Hre is the target Angle: ", targAngle);
        telemetry.addData("Here is the current Angle: ", angle.firstAngle);
        telemetry.addData("Here is the power being sent to the left side: ", TL.getPower());
        telemetry.addData("Here is the power being sent to the right side: ", TR.getPower());

        if (delta < -180){
            delta += 360;
        }
        else if (delta >180 )
            delta -= 360;

        globalAngle += delta;



        double gain = 0.1;
        double relativeError = (delta)/(targAngle);
        double correction = relativeError*gain;

        double leftPower;
        double rightPower;

        if (delta<0){
            leftPower = power - correction;
            rightPower = power + correction;
        }
        if (delta>0){
            leftPower = power + correction;
            rightPower = power - correction;
        }
        else {
            leftPower = power;
            rightPower = power;
        }

        TL.setPower(leftPower);
        BL.setPower(leftPower);
        TR.setPower(rightPower);
        BR.setPower(rightPower);
        telemetry.update();


    }

/*    public void resetAngle(){
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double getAngle(){
        Orientation currentAngle = new Orientation();
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return currentAngle.firstAngle;
    }

    public double correction(int targetAngle) {
        double gain = 0.10;
        double correction = getAngle() -targetAngle;
        correction = correction*gain;

        return correction;
    }*/

}
