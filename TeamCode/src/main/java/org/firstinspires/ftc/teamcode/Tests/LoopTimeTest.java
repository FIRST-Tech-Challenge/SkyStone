package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.lynx.commands.core.*;

@Autonomous
public class LoopTimeTest extends LinearOpMode {


    // declaring all hardware objects
    private BNO055IMU imu;
    private DcMotorEx m1;
    private DcMotorEx m2;
    private DcMotorEx m3;
    private DcMotorEx m4;

    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotorEx.class,"motorFrontRight");
        m2 = hardwareMap.get(DcMotorEx.class,"motorFrontLeft");
        m3 = hardwareMap.get(DcMotorEx.class,"motorBackRight");
        m4 = hardwareMap.get(DcMotorEx.class,"motorBackLeft");

        LynxModule module = hardwareMap.getAll(LynxModule.class).iterator().next();

        waitForStart();

        //code that executes in match

        ElapsedTime time = new ElapsedTime();
        double average = 0.0;
        double count = 0;

        while(opModeIsActive()){

            count += 1;
            LynxGetBulkInputDataResponse response;
            try {
                LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(module);
                response = command.sendReceive();
            }
            catch (Exception e) {
                telemetry.addData("Exception", "bulk read exception");
            }

            double randomPow = Math.random();
            m1.setPower(randomPow);
            m2.setPower(randomPow);
            m3.setPower(randomPow);
            m4.setPower(randomPow);

            average += (time.seconds() - average)/count;

            telemetry.addData("update frequency", 1.0/average);
            telemetry.update();

            time.reset();

        }


    }
}
