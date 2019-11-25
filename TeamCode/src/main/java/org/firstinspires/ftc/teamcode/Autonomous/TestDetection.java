package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.All.HardwareMap;

@Autonomous(name="Test Encoder", group="Test")       //Dashboard: https://192.168.49.1:8080/dash
@Disabled
public class  TestDetection extends LinearOpMode {
    private static double p, i, d, f = 0.0;

    @Override
    public void runOpMode(){
        HardwareMap map = new HardwareMap(hardwareMap);

        map.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        map.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        map.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        map.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(150);

        map.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        map.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        map.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        map.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("STATUS", "Ready for START!");
        telemetry.update();

        waitForStart();

        PIDFCoefficients PIDCoefficients = new PIDFCoefficients(p, i, d, f);
        map.frontLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients);
        map.frontRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients);
        map.backLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients);
        map.backRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDCoefficients);

        while(opModeIsActive()){
            telemetry.addData("FrontLeft", map.frontLeft.getCurrentPosition());
            telemetry.addData("FrontRight", map.frontLeft.getCurrentPosition());
            telemetry.addData("BackLeft", map.frontLeft.getCurrentPosition());
            telemetry.addData("BackRight", map.frontLeft.getCurrentPosition());
        }
    }
}
