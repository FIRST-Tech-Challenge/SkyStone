package org.firstinspires.ftc.teamcode.PreseasonTest;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.All.HardwareMap;

@Autonomous(name="Test Config", group="Test")       //Dashboard: https://192.168.49.1:8080/dash
public class TestWheels extends LinearOpMode {
    @Override
    public void runOpMode(){
        HardwareMap map = new HardwareMap(hardwareMap);
        waitForStart();

        map.frontLeft.setPower(.5);
        map.frontRight.setPower(-.5);
        map.backLeft.setPower(.5);
        map.backRight.setPower(-.5);

        sleep(1000);

        map.frontLeft.setPower(0);
        map.frontRight.setPower(0);
        map.backLeft.setPower(0);
        map.backRight.setPower(0);

        while(opModeIsActive()) {
            telemetry.addData("LeftForward: ", map.leftForward.getConnectionInfo());
            telemetry.addData("RightForward: ", map.rightForward.getConnectionInfo());
            telemetry.addData("Sideways: ", map.sideways.getConnectionInfo());
        }
    }
}
