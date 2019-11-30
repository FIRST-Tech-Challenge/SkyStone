package org.firstinspires.ftc.teamcode.Skystone.Auto;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
import org.firstinspires.ftc.teamcode.Skystone.Vision;

public class AutoBase extends LinearOpMode {
    protected Robot robot;
    protected Vision vision;
    protected long currentTime;
    Position2D position2D;
    public void initLogic(){
        //Init's robot
        robot = new Robot(this.hardwareMap, this.telemetry, this);
        vision = new Vision(this);

        robot.driveMotorsBreakZeroBehavior();
        initServos();

        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        position2D = new Position2D(robot);
    }

    private void initServos() {
        robot.getMarkerServo().setPosition(robot.TEAM_MARKER_RETRACT);
        robot.getBackStopper().setPosition(robot.BACK_STOPPER_UP);
        boolean isRetract = true;
        long outtakeExecutionTime = 0;
        long currentTime;

        robot.foundationMover(false);
        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED);
        robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);

        while (isRetract && robot.getLinearOpMode().opModeIsActive()) {
            currentTime = SystemClock.elapsedRealtime();
            if (currentTime - outtakeExecutionTime >= 250 && isRetract) {
                robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
            }
            if (currentTime - outtakeExecutionTime >= 950 && isRetract) {
                robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
                isRetract = false;
            }
        }
    }

    @Override
    public void runOpMode() {}

    protected void depositStone() {
        boolean isExtend = true;
        long outtakeExecutionTime = SystemClock.elapsedRealtime();

        robot.getIntakePusher().setPosition(robot.PUSHER_PUSHED); // Push block all the way to clamp

        while(isExtend && robot.getLinearOpMode().opModeIsActive()) {
            currentTime = SystemClock.elapsedRealtime();
            //extend
            //extend
            if (currentTime - outtakeExecutionTime >= 500 && isExtend) {
                robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED);
            }
            if (currentTime - outtakeExecutionTime >= 850 && isExtend) {
                robot.getClamp().setPosition(robot.CLAW_SERVO_CLAMPED);
            }
            if(currentTime-outtakeExecutionTime >= 1000 && isExtend){
                robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_EXTENDED);
            }

            if(currentTime-outtakeExecutionTime >= 1750 && isExtend){
                robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_90);
            }

            if(currentTime-outtakeExecutionTime >=2250){
                isExtend = false;
            }
        }
    }

    protected void retractOuttakeWait() {
        robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED); // Push block all the way to clamp
        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED); // Release clamp

        sleep(450);

        robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_RETRACTED);

        sleep(1150-450);

        robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
    }

    protected void retractOuttake() {
        boolean isRetract = true;
        long outtakeExecutionTime = SystemClock.elapsedRealtime();

        robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED); // Push block all the way to clamp
        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED); // Release clamp

        while(isRetract && robot.getLinearOpMode().opModeIsActive()) {
            currentTime = SystemClock.elapsedRealtime();

            if(currentTime-outtakeExecutionTime >= 450 && isRetract){
                robot.getClampPivot().setPosition(robot.OUTTAKE_PIVOT_RETRACTED);
            }
            if(currentTime-outtakeExecutionTime >= 1150 && isRetract){
                robot.getOuttakeExtender().setPosition(robot.OUTTAKE_SLIDE_RETRACTED);
                isRetract = false;
            }
        }
    }


    protected void intake(boolean intake) {
        if (intake) {
            robot.getIntakeLeft().setPower(1);
            robot.getIntakeRight().setPower(1);
        } else {
            robot.getIntakeLeft().setPower(0);
            robot.getIntakeRight().setPower(0);
        }
    }
}



