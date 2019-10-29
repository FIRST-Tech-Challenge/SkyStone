package org.firstinspires.ftc.teamcode.testClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;

@Autonomous
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Chassis chassis = new Chassis(hardwareMap);
        while (opModeIsActive()) {
            chassis.runDistance(10,0,2,1);
            while(chassis.runDistanceCheck(10))
            telemetry.update();
        }
    }
}