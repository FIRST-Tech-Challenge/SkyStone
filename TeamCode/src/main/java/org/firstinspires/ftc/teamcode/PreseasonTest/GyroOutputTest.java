package org.firstinspires.ftc.teamcode.PreseasonTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.All.HardwareMap;

@TeleOp(name="GyroTester", group="Linear Opmode")
@Disabled
public class GyroOutputTest extends LinearOpMode {
    boolean a = false;
    boolean b = false;
    boolean x = false;
    @Override
    public void runOpMode() throws InterruptedException{
        HardwareMap map = new HardwareMap(hardwareMap);
        telemetry.addData("Instructions:","A to turn clockwise");
        telemetry.addData("Instructions:","B to turn counter clockwise");
        telemetry.addData("Instructions:","X to Display telemetry");
        telemetry.update();
        map.gyroInit();
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                b = false;
                map.frontLeft.setPower(0.5);
                map.frontRight.setPower(0.5);
                map.backLeft.setPower(0.5);
                map.backRight.setPower(0.5);
                a = true;
            } else {
                map.frontLeft.setPower(0);
                map.frontRight.setPower(0);
                map.backLeft.setPower(0);
                map.backRight.setPower(0);
                a = false;
            }
            if(gamepad1.b){
                a = false;
                map.frontLeft.setPower(-0.5);
                map.frontRight.setPower(-0.5);
                map.backLeft.setPower(-0.5);
                map.backRight.setPower(-0.5);
                b = true;
            } else {
                map.frontLeft.setPower(0);
                map.frontRight.setPower(0);
                map.backLeft.setPower(0);
                map.backRight.setPower(0);
                b = false;
            }
            if(gamepad1.x && !x){
                telemetry.clear();
                telemetry.addData(map.TAG,"Data will now be shown");
                telemetry.update();
                x = true;
            } else if(gamepad1.x && x){
                telemetry.addData(map.TAG,"Data turned off");
                telemetry.update();
                x = false;
            }
            if(x){
                telemetry.clear();
                telemetry.addData(map.TAG, "Angle: "+map.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.update();
            }
            if(gamepad1.y) {
            }
        }
    }
}
