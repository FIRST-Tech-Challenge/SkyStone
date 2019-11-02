package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Loading Zone Red")
public class MM_LoadingZoneRed extends LinearOpMode {

    private Robot robot = new Robot();
    enum Skystone {LEFT, CENTER, RIGHT}
    private Skystone skystonePos = Skystone.LEFT;
    private double distanceToBuildZone = 0.0; // distance to skybridge from close edge of block
    private double speed = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        // Detect skystone with camera
        int position = robot.detectSkystone(this);
        if (position == -1) {
            skystonePos = Skystone.LEFT;
        } else if (position == 0) {
            skystonePos = Skystone.CENTER;
        } else {
            skystonePos = Skystone.RIGHT;
        }

        // wait for start
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
        /* Pick Block up with arm */
        robot.pickUpBlock(this);
        Thread.sleep(500);
        // back up
        robot.driveForwardDistance(6, -speed, this);
        // turn towards skybridge
        robot.turnRight(speed, 650);
        // drive to skybridge
        robot.driveForwardDistance(distanceToBuildZone + 12, speed, this);
        Thread.sleep(500);
        // drop block
        robot.releaseBlock(this);
        // fold arm back
        robot.foldArmBack(this);
        Thread.sleep(500);
        // park
        robot.driveForwardDistance(12, -speed, this);



    }



}
