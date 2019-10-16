package org.firstinspires.ftc.teamcode.PreseasonTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.All.HardwareMap;

@Autonomous(name="Test Config", group="Test")       //Dashboard: https://192.168.49.1:8080/dash
public class TestWheels extends LinearOpMode {
    @Override
    public void runOpMode(){
        HardwareMap map = new HardwareMap(hardwareMap);

        HardwareMap.track.resetEncoders();
        HardwareMap.track.encoders(true,25,25);

        telemetry.addData("STATUS", "Ready for START!");
        telemetry.update();

        waitForStart();

        map.frontLeft.setPower(.5);
        map.frontRight.setPower(-.5);
        map.backLeft.setPower(.5);
        map.backRight.setPower(-.5);

        sleep(500);

        map.frontLeft.setPower(0);
        map.frontRight.setPower(0);
        map.backLeft.setPower(0);
        map.backRight.setPower(0);

        while(opModeIsActive()){
            telemetry.addData("LeftForward", map.leftForward.getVoltage());
            telemetry.addData("RightForward", map.rightForward.getVoltage());
            telemetry.addData("Sideways", map.sideways.getVoltage());

            telemetry.addData("LeftForward", HardwareMap.track.getEncoderTicks().get(0));
            telemetry.addData("RightForward", HardwareMap.track.getEncoderTicks().get(1));
            telemetry.addData("Sideways", HardwareMap.track.getEncoderTicks().get(2));

            telemetry.addData("TrackingTime:",HardwareMap.track.getElapsedTime().get(0) + ", " +
                    HardwareMap.track.getElapsedTime().get(1));

            telemetry.addData("Formula", HardwareMap.track.getEncoderDebug().get(0));
            telemetry.addData("LeftDebug", HardwareMap.track.getEncoderDebug().get(1));
            telemetry.addData("RightDebug", HardwareMap.track.getEncoderDebug().get(2));
            telemetry.addData("SidewaysDebug", HardwareMap.track.getEncoderDebug().get(3));
            telemetry.update();
        }
    }
}
