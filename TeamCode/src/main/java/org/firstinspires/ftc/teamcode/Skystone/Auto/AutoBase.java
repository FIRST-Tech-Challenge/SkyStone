package org.firstinspires.ftc.teamcode.Skystone.Auto;


import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

public class AutoBase extends LinearOpMode {
    public Robot robot;
    protected long currentTime;
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
    public void runOpMode() {

    }

    protected void extendOuttake(Robot robot) {
        boolean isExtend = true;
        long outtakeExecutionTime = SystemClock.elapsedRealtime();

        robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED); // Push block all the way to clamp

        while(isExtend) {
            currentTime = SystemClock.elapsedRealtime();
            //extend
            if (currentTime - outtakeExecutionTime >= 300) {
                robot.getClamp().setPosition(robot.CLAW_SERVO_CLAMPED);
            }
            if(currentTime-outtakeExecutionTime >= 400){
                robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
            }
            if(currentTime-outtakeExecutionTime >= 1750){
                robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_EXTENDED);
            }
            if(currentTime-outtakeExecutionTime >=250){
                isExtend = false;
            }
        }
    }

    protected void retractOuttake(Robot robot) {
        boolean isRetract = true;
        long outtakeExecutionTime = SystemClock.elapsedRealtime();

        robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED); // Push block all the way to clamp
        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED); // Release clamp

        while(isRetract) {
            currentTime = SystemClock.elapsedRealtime();

            //retract
            if (currentTime - outtakeExecutionTime >= 250) {
                robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
            }
            if (currentTime - outtakeExecutionTime >= 750) {
                robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
                isRetract = false;
            }
        }
    }

}



