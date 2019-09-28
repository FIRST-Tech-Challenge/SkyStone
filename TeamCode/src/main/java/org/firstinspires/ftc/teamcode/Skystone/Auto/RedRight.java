package org.firstinspires.ftc.teamcode.Skystone.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.PathPoints;
import org.firstinspires.ftc.teamcode.Skystone.MotionProfiler.Point;
import org.firstinspires.ftc.teamcode.Skystone.Odometry.Position2D;
import org.firstinspires.ftc.teamcode.Skystone.Robot;

@Autonomous(name="RedLeft", group ="Concept")
public class RedRight extends AutoBase{
    @Override
    public void runOpMode() {
        super.runOpMode();
            Robot robot = new Robot(hardwareMap,telemetry,this);
            robot.driveMotorsBreakZeroBehavior();
            robot.setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            waitForStart();

            Position2D position2D = new Position2D(robot);
            position2D.startOdometry();
            PathPoints testPath = new PathPoints(new double[][] {{20,0}}, 9);

        while (opModeIsActive()){
            robot.goToPoint(20, 0, 0.5, 0.5, 0);
            sleep(100);
            boolean detectedStone = false;
            Point point = new Point(0,0);
            while (!detectedStone) {
                if (robot.detectSkystone().equals(true)){
                    point = robot.detectSkystone();
                    detectedStone = true;
                }
            }
            robot.goToPoint(point.x, point.y, 0.5, 0.5, 0);
            telemetry.addData("Skystone Location",robot.detectSkystone());
            //robot.moveFollowCurve(testPath.targetPoints);
            sleep(100000);
        }
    }
}
