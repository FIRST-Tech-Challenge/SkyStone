package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "LoudGoesTheBoom", group = "nameCalling")
public class IMUHelper extends LinearOpMode {
    public BNO055IMU imu;
    public Orientation angles;

    public DcMotor TL;
    public DcMotor TR;
    public DcMotor BL;
    public DcMotor BR;

    Orientation lastAngle;

    public double globalAngle;

    public double leftPower, rightPower;
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        TL = hardwareMap.get(DcMotor.class, "TL");
        TR = hardwareMap.get(DcMotor.class, "TR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        while(opModeIsActive()){
            setPower(leftPower, rightPower);

            telemetry.addData("Here is the angle in the Z: ", getAngle());
            telemetry.update();
        }
    }

    public void resetAngle() {
        lastAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.globalAngle = 0;
    }

    public void setPower(double leftPower, double rightPower) {
        TL.setPower(leftPower);
        BL.setPower(leftPower);
        TR.setPower(rightPower);
        BR.setPower(rightPower);
    }

    public double getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public void straightLine(int time) {
        /*
        establish the required angle
         */

        resetAngle(); //gets the angle required
        Orientation currentAngle = new Orientation();
        currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle;

        deltaAngle = currentAngle.firstAngle - lastAngle.firstAngle;
        telemetry.addData("Here is the deltaAngle: ", deltaAngle);
        telemetry.addData("Here is the currentAngle: ", currentAngle.firstAngle);
        telemetry.addData("Here is the lastAngle: ", lastAngle.firstAngle);
        telemetry.update();
    }

    public void straightLine() {

    }

}
