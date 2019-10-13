package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "MMAutoTest")
public class MMAutonomous extends LinearOpMode {

    Robot robot = new Robot();
    enum Skystone {LEFT, CENTER, RIGHT}
    Skystone skystonePos = Skystone.LEFT;
    double distanceToBuildZone = 0.0; // distance to skybridge from close edge of block
    double speed = 0.25;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        // Drive to quarry
        robot.driveForwardDistance(47.0, speed);
        switch (skystonePos) {
            case LEFT:
                distanceToBuildZone = 32;
                // strafe to block
                robot.strafeTime(-speed, 500);
                break;
            case CENTER:
                distanceToBuildZone = 28;
                break;
            case RIGHT:
                distanceToBuildZone = 24;
                // strafe to block
                robot.strafeTime(speed, 500);
                break;
        }

        // back up
        robot.driveForwardDistance(-6, speed);
        // turn towards skybridge
        //robot.turnRight(speed, 2000);
        // drive to skybridge
        robot.driveForwardDistance(distanceToBuildZone + 6, speed);
        // park
        robot.driveForwardDistance(-6, speed);



    }

}
