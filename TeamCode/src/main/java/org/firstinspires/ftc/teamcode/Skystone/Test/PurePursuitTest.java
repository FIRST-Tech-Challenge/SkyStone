package org.firstinspires.ftc.teamcode.Skystone.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.SplineInterpolate;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Autonomous(name="PurePursuitTest ", group="Linear Opmode")
public class PurePursuitTest  extends LinearOpMode
{
    public static double[][] testPoints = {{0,20},{20,-20}};

    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();
        robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
//        robot.intializeIMU();
//        robot.changeRunModeToUsingEncoder();
//        while(!gamepad1.a){
//            if(gamepad1.dpad_up) {
//                testPath.followDistance+=0.01;
//            }
//            if(gamepad1.dpad_down){
//                testPath.followDistance-=0.01;
//            }
//            sleep(50);
//            telemetry.addLine(Double.toString(testPath.followDistance));
//            telemetry.update();
//        }
//        testPath.newPoints(testPoints);

//            robot.followCurve(testPath.targetPoints,Math.toRadians(0));
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
        PathPoints testPath = new PathPoints(testPoints, 9);

        while (opModeIsActive()){
//            robot.followCurve(testPath.targetPoints,Math.toRadians(0));
            robot.moveFollowCurve(testPath.targetPoints);
            sleep(5000);
            break;
        }
    }
}