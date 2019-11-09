package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Autonomous-Block-Blue", group="Linear Opmode")
public class AutonomousBlockBlue extends Movement {

    @Override
    public void runOpModeImpl() {

        waitForStart();

        //Servo goes up
        frontServo.setPosition(0.4);
        sleep(200);

        // moveBlocksOld();
        moveThreeBlocks();
    }

    private void moveThreeBlocks() {

       // While returning we come faster so rightDuration will be smaller
        moveBlock(1, 0.5, 1750, 1250,
                2900, 0.0, 3500, 0.0);

        moveBlock(2, 0.5, 1375, 1100,
                2750, 0.2, 3900, 0.0);

        moveBlock(3, 0.5, 1500, 1250,
                0, 0.0, 4200, 0.0);
    }



    private void moveBlock(final int blockNumber,
                           final double wheelPower,
                           final int forwardDuration,
                           final int backwardDuration,
                           final int leftDuration,
                           final double calibrationPower,
                           final int rightDuration,
                           final double calibrationPower2) {
        // Move Forward towards the blocks

        arm.setPower(-0.3);
        sleep(200);
        arm.setPower(0.0);
        goForward(wheelPower, forwardDuration, "Going forward");
        stop("Pausing");
        sleep(50);

        //Servo goes down, to grab the block
        frontServo.setPosition(0.0);

        //robot moves backward
        goBackward(wheelPower, backwardDuration, "Going backward");


        stop("Pausing");

        /*if (blockNumber == 3) {
            calibrateDirection(0.4, 300);
        }*/

        //move right - towards the depot
        goRight(wheelPower, rightDuration, "Going right");
        stopWithSleep("Pausing", 300);

        calibrateDirection(calibrationPower, 200);

        //servo up, release the block
        frontServo.setPosition(0.4);
        stop("Pausing");

        //move left
        goLeft(1, leftDuration, "Going left");
        stopWithSleep("Pausing", 300);

        calibrateDirection(calibrationPower2, 300);

    }

    private void calibrateDirection(final double leftWheelPower, final int duration) {
        leftfront.setPower(leftWheelPower);
        leftback.setPower(leftWheelPower);
        rightfront.setPower(0);
        rightback.setPower(0);
        sleep(duration);
    }


}