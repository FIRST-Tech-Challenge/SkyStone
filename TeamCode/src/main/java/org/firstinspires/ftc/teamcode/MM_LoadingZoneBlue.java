package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Loading Zone Blue")
public class MM_LoadingZoneBlue extends LinearOpMode {

    Robot robot = new Robot();
    enum Skystone {LEFT, CENTER, RIGHT}
    Skystone skystonePos = Skystone.LEFT;
    double distanceToBuildZone = 0.0; // distance to skybridge from close edge of block
    double speed = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        // Drive to quarry
        robot.driveForwardDistance(47.0 - robot.ROBOT_EXTENDED_LENGTH, speed, this);
        Thread.sleep(500);
        switch (skystonePos) {
            case LEFT:
                distanceToBuildZone = 32 - robot.ROBOT_EXTENDED_LENGTH;
                // strafe to block
                robot.strafeTime(-speed, 2000);
                break;
            case CENTER:
                distanceToBuildZone = 28 - robot.ROBOT_EXTENDED_LENGTH;
                break;
            case RIGHT:
                distanceToBuildZone = 24 - robot.ROBOT_EXTENDED_LENGTH;
                // strafe to block
                robot.strafeTime(speed, 2000);
                break;
        }

        Thread.sleep(500);
        // back up
        robot.driveForwardDistance(6, -speed, this);
        // turn towards skybridge
        robot.turnRight(speed, 650);
        // drive to skybridge
        robot.driveForwardDistance(distanceToBuildZone + 6, speed, this);
        // park
        robot.driveForwardDistance(6, -speed, this);



    }



}
