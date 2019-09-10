package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.Auto.TensorFlowMineralDetection;
import org.firstinspires.ftc.teamcode.RoverRuckus.RR2.RR2;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class AutoBase extends LinearOpMode {
    public Robot robot;

    public void initLogic(){
        //Init's robot
        robot = new Robot(hardwareMap,telemetry,this);

        robot.driveMotorsBreakZeroBehavior();

        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intializeIMU();
        waitForStart();

        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
    }

    @Override
    public void runOpMode(){
    }
}


