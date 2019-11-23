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

    public void initLogic(){
        vision.initVision();

        //Init's robot
        robot = new Robot(this.hardwareMap, this.telemetry, this);

        robot.driveMotorsBreakZeroBehavior();

        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //robot.intializeIMU();
        //initServos();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
    }

    private void initServos() {
        boolean isRetract = true;
        long outtakeExecutionTime = 0;
        long currentTime;
        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED);

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

    protected void retractOuttake() {
        boolean isRetract = true;
        long outtakeExecutionTime = SystemClock.elapsedRealtime();

        robot.getIntakePusher().setPosition(robot.PUSHER_RETRACTED); // Push block all the way to clamp
        robot.getClamp().setPosition(robot.CLAW_SERVO_RELEASED); // Release clamp

        while(isRetract&& robot.getLinearOpMode().opModeIsActive()) {
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


    public void goToSkystone(int skystone, int robotPosition){
        final String VUFORIA_KEY = "AbSCRq//////AAAAGYEdTZut2U7TuZCfZGlOu7ZgOzsOlUVdiuQjgLBC9B3dNvrPE1x/REDktOALxt5jBEJJBAX4gM9ofcwMjCzaJKoZQBBlXXxrOscekzvrWkhqs/g+AtWJLkpCOOWKDLSixgH0bF7HByYv4h3fXECqRNGUUCHELf4Uoqea6tCtiGJvee+5K+5yqNfGduJBHcA1juE3kxGMdkqkbfSjfrNgWuolkjXR5z39tRChoOUN24HethAX8LiECiLhlKrJeC4BpdRCRazgJXGLvvI74Tmih9nhCz6zyVurHAHttlrXV17nYLyt6qQB1LtVEuSCkpfLJS8lZWS9ztfC1UEfrQ8m5zA6cYGQXjDMeRumdq9ugMkS";

        // For all moveToPoints (because they are relative to the starting position of the robot),
        // robotPosition is used to identify whether to go for the skystone in the first set or the second set.
        // If robotPosition is negative, then the second set of stones is to the left of the original position of the robot.

        robot.moveToPoint(11.5 ,robotPosition * 24,0.4,1,Math.toRadians(0));
        telemetry.addLine("done with move");
        telemetry.update();

        telemetry.addLine("go to point");
        int position = 0;

        //intake(true);

        if (skystone == 2) {
            telemetry.addLine("left");
            robot.moveToPoint(10, 3 + (robotPosition * 24), 0.55, 0.5, Math.toRadians(0));
        } else if (skystone == 0){
            telemetry.addLine("right");
            robot.moveToPoint(10, -3 + (robotPosition * 24), 0.55, 0.5, Math.toRadians(0));
        } else {
            telemetry.addLine("center");
            robot.moveToPoint(10, robotPosition * 24, 0.55, 0.5, Math.toRadians(0));
        }
        telemetry.addLine("Done with detect");
        telemetry.update();

        //intake(false);

        telemetry.addLine("Done with move");
        telemetry.update();

    }
}



