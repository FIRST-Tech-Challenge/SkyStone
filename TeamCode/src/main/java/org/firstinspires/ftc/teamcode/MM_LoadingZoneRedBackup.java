package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Loading Zone Red")
public class MM_LoadingZoneRedBackup extends LinearOpMode {

    private Robot robot = new Robot();
    enum Skystone {LEFT, CENTER, RIGHT}
    private Skystone skystonePos = Skystone.LEFT;
    private enum ParkingPosition {FAR, CLOSE}// far or close to center
    private ParkingPosition parkingPosition = ParkingPosition.CLOSE;
    private double distanceToBuildZone = 0.0; // distance to skybridge from close edge of block
    private double speed = 0.4;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this);
        robot.releaseBlock(this);
        // Detect skystone with camera
        int position;
        while (true) {
            position = robot.detectSkystone(this);
            if (position == -1) {
                skystonePos = Skystone.LEFT;
                break;
            } else if (position == 0) {
                skystonePos = Skystone.CENTER;
                break;
            } else if (position == 1) {
                skystonePos = Skystone.RIGHT;
                break;
            }
            idle();
        }

        // wait for start
        waitForStart();

        // put arm down
        robot.bringArmDown(this);
        robot.rotateGripper(0.8);

        // Drive to quarry
        robot.driveForwardDistance(17, speed, this);
        Thread.sleep(500);
        switch (skystonePos) {
            case LEFT:
                distanceToBuildZone = 32;
                // strafe to block
                robot.strafeTime(-speed, 1000);
                break;
            case CENTER:
                distanceToBuildZone = 28;
                robot.strafeTime(-speed, 250);
                break;
            case RIGHT:
                distanceToBuildZone = 24;
                // strafe to block
                robot.strafeTime(speed, 100);
                break;

        }
        Thread.sleep(500);
        // grab block
        robot.rotateGripper(0.5);
        Thread.sleep(500);
        robot.gripBlock();
        Thread.sleep(500);
        robot.rotateGripper(0.8);
        // back up
        robot.driveForwardDistance(8, -speed, this);
        // turn towards skybridge
        robot.turnRight(speed, 1075);
        // drive to skybridge
        robot.driveForwardDistance(distanceToBuildZone + 30, speed, this);
        Thread.sleep(500);
        // drop block
        robot.rotateGripper(0.5);
        Thread.sleep(100);
        robot.releaseBlock(this);
        Thread.sleep(100);
        robot.rotateGripper(1.0);
        // fold arm back
        //robot.foldArmBack(this);
        Thread.sleep(500);
        // park
        robot.driveForwardDistance(28, -speed, this);

        if (parkingPosition == parkingPosition.CLOSE) {
            robot.strafeTime(-speed, 2800);
        }



    }



}
