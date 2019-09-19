package org.firstinspires.ftc.teamcode.competition.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.competition.Hardware;
import org.firstinspires.ftc.teamcode.competition.MecanumDrive;
import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;

/**
 * Testing out the odometry
 */
@Autonomous(name = "OdometryTest", group = "Auto")
public class OdometryTest extends LinearOpMode {

    private Hardware robot;
    private MecanumDrive driveTrain;

    private enum TestAuto {
        FORWARD_CM,
        FORWARD_CM_PID,
        MOVE_TO_POINT
    }

    @Override
    public void runOpMode() {
        // Sets up classes
        robot = new Hardware();
        driveTrain = new MecanumDrive(robot);

        TestAuto autoToRun = TestAuto.FORWARD_CM;

        // Initializes robot
        robot.init(hardwareMap);

        // Waits until "start" or "stop" is pressed
        while(!isStarted()&&!isStopRequested()){
            if(gamepad1.x)
                autoToRun = TestAuto.FORWARD_CM;
            if(gamepad1.a)
                autoToRun = TestAuto.FORWARD_CM_PID;
            if(gamepad1.b)
                autoToRun = TestAuto.MOVE_TO_POINT;

            telemetry.addData("auto to run", autoToRun);
            telemetry.update();
        }

        // If testing new driveTrain.forwardCm(cm)
        if(autoToRun == TestAuto.FORWARD_CM){
            // Testing new driveTrain.forwardCm(30) since it doesn't work
            double endpointX = robot.x + 30;

            // Create variables for if robot is stuck and won't move forward
            int timer = 0;
            double lastStuckX = robot.x;

            // Loop to try and move robot forward
            while(opModeIsActive() && (robot.x < endpointX)) {
                // If robot hasn't moved within last 1000 updates
                if (timer >= 1000)
                    break;

                // Try and move robot forward
                driveTrain.drive(1,0,0);

                // Check if robot is stuck in same position
                if(HelperMethods.inThreshhold(lastStuckX, robot.x, .5))
                    timer++;
                else {
                    lastStuckX = robot.x;
                    timer = 0;
                }

                telemetry.addData("X", robot.x);
                telemetry.addData("Y", robot.y);
                telemetry.addData("Theta", robot.theta);
                telemetry.update();
            }

            driveTrain.powerSet(0);
        }

        // If testing new driveTrain.forwardCmPid(cm, tolerance)
        else if (autoToRun == TestAuto.FORWARD_CM_PID){
            // Testing new driveTrain.forwardCmPid(30)
            double endpointX = robot.x + 30;
            double power = 1;

            while(opModeIsActive() && (robot.x < endpointX && power != 0)){
                if(robot.x - 30 <= 5)
                    power = (robot.x - 30) * .2;
                driveTrain.drive(power,0,0);

                telemetry.addData("X", robot.x);
                telemetry.addData("Y", robot.y);
                telemetry.addData("Theta", robot.theta);
                telemetry.update();
            }

            driveTrain.powerSet(0);
        }

        // If testing new driveTrain.moveToPoint(x, y, theta)
        else {

        }
    }
}
