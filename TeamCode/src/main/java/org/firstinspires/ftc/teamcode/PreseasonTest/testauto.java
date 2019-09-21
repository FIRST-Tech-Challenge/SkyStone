package org.firstinspires.ftc.teamcode.PreseasonTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.All.DriveConstant;
import org.firstinspires.ftc.teamcode.All.HardwareMap;

@Autonomous(name="Test Auto", group="Test")

public class testauto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        HardwareMap map = new HardwareMap(hardwareMap);
        map.resetEncoders();
        map.gyroInit();

        waitForStart();

        encoderDriveRotation(0.3,2);
        sleep(2000);
        gyroTurn(0.5,90,true);
        sleep(2000);
    }

    public void encoderDriveRotation(double power, double rotations){
        HardwareMap map = new HardwareMap(hardwareMap);
        double initFrontLeft=map.frontLeft.getCurrentPosition();
        double initFrontRight=map.frontRight.getCurrentPosition();
        double initBackLeft=map.backLeft.getCurrentPosition();
        double initBackRight=map.backLeft.getCurrentPosition();
        double q;
        double w;
        double e;
        double r;
        map.frontLeft.setPower(power);
        map.frontRight.setPower(-power);
        map.backLeft.setPower(power);
        map.backRight.setPower(-power);
        while(opModeIsActive() && !isStopRequested()) {
            if (initFrontLeft - map.frontLeft.getCurrentPosition() <= rotations * -DriveConstant.ENCODER_COUNTS_PER_REVOLUTION &&
                    initFrontRight - map.frontRight.getCurrentPosition() >= rotations * DriveConstant.ENCODER_COUNTS_PER_REVOLUTION &&
                    initBackLeft - map.backLeft.getCurrentPosition() <= rotations * -DriveConstant.ENCODER_COUNTS_PER_REVOLUTION &&
                    initBackRight - map.backRight.getCurrentPosition() >= rotations * DriveConstant.ENCODER_COUNTS_PER_REVOLUTION) {
                map.frontLeft.setPower(0);
                map.frontRight.setPower(0);
                map.backLeft.setPower(0);
                map.backRight.setPower(0);
                break;
            }
            q = initFrontRight - map.frontRight.getCurrentPosition();
            w = initFrontLeft - map.frontLeft.getCurrentPosition();
            e = initBackLeft - map.backLeft.getCurrentPosition();
            r = initBackRight - map.backRight.getCurrentPosition();
            telemetry.clear();
            telemetry.addData(map.TAG,"frontRight: "+ q + " >= " + rotations * DriveConstant.ENCODER_COUNTS_PER_REVOLUTION);
            telemetry.addData(map.TAG,"frontLeft: "+ w + " <= " + rotations * -DriveConstant.ENCODER_COUNTS_PER_REVOLUTION);
            telemetry.addData(map.TAG,"backRight: "+ r + " >= " + rotations * DriveConstant.ENCODER_COUNTS_PER_REVOLUTION);
            telemetry.addData(map.TAG,"backLeft: "+ e + " <= " + rotations * -DriveConstant.ENCODER_COUNTS_PER_REVOLUTION);
            telemetry.update();
        }
    }
    public void gyroTurn(double power, double angle, boolean clockwise){
        HardwareMap map = new HardwareMap(hardwareMap);
        if(clockwise){
            map.frontLeft.setPower(power);
            map.frontRight.setPower(power);
            map.backLeft.setPower(power);
            map.backRight.setPower(power);
            angle = -angle;
        } else {
            map.frontLeft.setPower(-power);
            map.frontRight.setPower(-power);
            map.backLeft.setPower(-power);
            map.backRight.setPower(-power);
        }
        while(opModeIsActive() && !isStopRequested()) {
            telemetry.clear();
            telemetry.addData(map.TAG,"Angle: "+ map.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
            if(map.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle >= angle - 15 && !clockwise){
                map.frontLeft.setPower(0);
                map.frontRight.setPower(0);
                map.backLeft.setPower(0);
                map.backRight.setPower(0);
                break;
            } else if(map.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle <= angle + 15 && clockwise){
                map.frontLeft.setPower(0);
                map.frontRight.setPower(0);
                map.backLeft.setPower(0);
                map.backRight.setPower(0);
                break;
            }
        }
    }

}
