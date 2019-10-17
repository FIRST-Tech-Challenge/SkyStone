package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Loading Zone Blue")
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
        robot.driveForwardDistance(47.0, speed, this);
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
        robot.driveForwardDistance(6, -speed, this);
        // turn towards skybridge
        robot.turnRight(speed, 1400);
        // drive to skybridge
        robot.driveForwardDistance(distanceToBuildZone + 6, speed, this);
        // park
        robot.driveForwardDistance(6, -speed, this);



    }
<<<<<<< HEAD
=======
    public void driveForwardDistance(double distance, double power) {
        //* drives forward a certain distance(in) using encoders *//*

        // calculate ticks
        long NUM_TICKS_LONG = StrictMath.round(robot.TICKS_PER_INCH * distance);
        int NUM_TICKS = (int) NUM_TICKS_LONG;

        // reset encoders
        robot.setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set target position
        robot.setDriveTargetPos(NUM_TICKS);

        // Set to RUN_TO_POSITION mode
        robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set drive power
        robot.setDrivePower(power);

        // while (this.rearLeft.isBusy() && this.frontLeft.isBusy() && this.rearRight.isBusy() && this.frontRight.isBusy()) {
        // wait until target position is reached
        //}

        while (opModeIsActive() && robot.rearLeft.isBusy())
        {
            telemetry.addData("encoder-fwd", robot.rearLeft.getCurrentPosition() + "  busy=" + robot.rearLeft.isBusy());
            telemetry.update();
            idle();
        }

        // stop driving
        robot.stopDrive();

        // set mode back to normal
        robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

>>>>>>> 5456ef24c17c9229b9b62a9c24cbeb5cf5c7789a

}
