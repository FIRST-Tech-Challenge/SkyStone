package org.firstinspires.ftc.teamcode.Skystone.Test;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;
@Autonomous(name="GoToPointTest", group="Linear Opmode")
public class GoToPointTest extends LinearOpMode
{
    @Override
    public void runOpMode(){
        Robot robot = new Robot(hardwareMap,telemetry,this);
        robot.driveMotorsBreakZeroBehavior();
        robot.resetEncoders();
        waitForStart();
        robot.intializeIMU();
        robot.changeRunModeToUsingEncoder();
        Position2D position2D = new Position2D(robot);
        position2D.startOdometry();
        while (opModeIsActive()){
            robot.goToPoint(15,0,1, 0.5,Math.toRadians(0));
            robot.goToPoint(0,-20,0.5, 0.5,Math.toRadians(0));
            robot.finalTurn(135);
            robot.goToPoint(60,-20,0.5, 0.9,Math.toRadians(0));
            robot.finalMove(1,-80);
            break;
        }
    }
}