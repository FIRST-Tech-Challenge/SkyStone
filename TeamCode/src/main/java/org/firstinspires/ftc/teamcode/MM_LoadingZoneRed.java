package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Loading Zone Red")
public class MM_LoadingZoneRed extends LinearOpMode {

    Robot robot = new Robot();
    enum Skystone {LEFT, CENTER, RIGHT}
    Skystone skystonePos = Skystone.LEFT;
    double distanceToBuildZone = 0.0; // distance to skybridge from close edge of block
    double speed = 0.4;

    // Gripper Test
    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    Servo servo;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;



    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        servo = hardwareMap.get(Servo.class, "left_hand");

        waitForStart();
        // detect sky stone with camera

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
        // slew the servo, according to the rampUp (direction) variable.
        if (rampUp) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
                rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
                rampUp = !rampUp;  // Switch ramp direction
            }
        }

        // Display the current value
        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

        // Set the servo to the new position and pause;
        servo.setPosition(position);
        sleep(CYCLE_MS);
        idle();

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
