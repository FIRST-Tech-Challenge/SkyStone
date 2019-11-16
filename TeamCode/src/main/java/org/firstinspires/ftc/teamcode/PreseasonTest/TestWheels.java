package org.firstinspires.ftc.teamcode.PreseasonTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.All.HardwareMap;

@Autonomous(name="Test Encoders", group="Test")       //Dashboard: https://192.168.49.1:8080/dash
public class TestWheels extends LinearOpMode {
    private static double power = 0.5;
    private static boolean blocked = false;

    @Override
    public void runOpMode(){
        HardwareMap map = new HardwareMap(hardwareMap);

        telemetry.addData("STATUS", "Ready for START!");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.y && power < 1 && !blocked) {
                power += 0.1;
                power = Math.round(power * 10.0) / 10.0;
                blocked = true;
            }
            if(gamepad1.a && power > 0.1 && !blocked) {
                power -= 0.1;
                power = Math.round(power * 10.0) / 10.0;
                blocked = true;
            }
            if(!gamepad1.y && !gamepad1.a)
                blocked = false;
            if(gamepad1.b){
                map.frontLeft.resetDeviceConfigurationForOpMode();
                map.frontRight.resetDeviceConfigurationForOpMode();
                map.backLeft.resetDeviceConfigurationForOpMode();
            }


            if(power < 0.1)
                power = 0.1;
            else if(power > 1)
                power = 1;

            if(gamepad1.left_stick_y == 1){
                map.frontLeft.setPower(power);
                map.backLeft.setPower(power);
            } else if(gamepad1.left_stick_y == -1){
                map.frontLeft.setPower(-power);
                map.backLeft.setPower(-power);
            } else {
                map.frontLeft.setPower(0);
                map.backLeft.setPower(0);
            }

            if(gamepad1.right_stick_y == 1){
                map.frontRight.setPower(-power);
                map.backRight.setPower(-power);
            } else if(gamepad1.right_stick_y == -1){
                map.frontRight.setPower(power);
                map.backRight.setPower(power);
            } else {
                map.frontRight.setPower(0);
                map.backRight.setPower(0);
            }

            telemetry.addData("Wheel Power", power);
            telemetry.addData("","-------------------");
            telemetry.addData("LeftForward", map.backLeft.getCurrentPosition());
            telemetry.addData("RightForward", map.frontRight.getCurrentPosition());
            telemetry.addData("Sideways", map.frontLeft.getCurrentPosition());

            /*telemetry.addData("LeftForward", HardwareMap.track.getEncoderTicks().get(0));
            telemetry.addData("RightForward", HardwareMap.track.getEncoderTicks().get(1));
            telemetry.addData("Sideways", HardwareMap.track.getEncoderTicks().get(2));

            telemetry.addData("TrackingTime:",HardwareMap.track.getElapsedTime().get(0) + ", " +
                    HardwareMap.track.getElapsedTime().get(1));

            telemetry.addData("","-------------------");
            telemetry.addData("Formula", HardwareMap.track.getEncoderDebug().get(0));
            telemetry.addData("LeftDebug", HardwareMap.track.getEncoderDebug().get(1));
            telemetry.addData("RightDebug", HardwareMap.track.getEncoderDebug().get(2));
            telemetry.addData("SidewaysDebug", HardwareMap.track.getEncoderDebug().get(3));*/

            telemetry.update();
        }
    }
}
