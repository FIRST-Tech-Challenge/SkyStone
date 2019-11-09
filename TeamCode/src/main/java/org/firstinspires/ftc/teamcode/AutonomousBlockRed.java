package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name="Autonomous-Block-Red", group="Linear Opmode")
public class AutonomousBlockRed extends Movement {

    @Override
    public void runOpModeImpl() {

        waitForStart();

        //TODO: Add the durations and the powers to everything

        // 1. Stone #1

        /*
        //Arm goes down
        arm.setPower(0.25);
        arm.setTargetPosition(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(0);
        sleep(100);

         */

        //Servo goes up
        frontServo.setPosition(0.4);
        sleep(200);

        // moveBlocksOld();

        // TODO: New refactored code needs testing. If working start using this code and remove moveBlocksOld()
        moveThreeBlocks();
    }

    private void moveThreeBlocks() {

       // While returning we come faster so rightDuration will be smaller
        moveBlock(1, 0.5, 1750, 1250,
                3500, 0.4, 2850, 0.2);

        moveBlock(2, 0.5, 1375, 1250,
                3900, 0.5, 3150, 0.2);

        moveBlock(3, 0.5, 1200, 1250,
                3900, 0.5, 100, 0.0);
    }


//    private void moveBlocksNewFaster() {
//        int leftDuration = 1900; // How much to move left to carry the block
//        final int blockIncrement = 375; // When we return back we have to move right extra to pick next block
//
//        for (int block = 1; block <= 5 ; block++) {
//            // When we return back we have to move right extra to pick next block
//            int rightDuration = leftDuration + blockIncrement;
//            int forwardDuration = block == 1 ? 1400: 1300; // Move extra for 1st block
//            moveBlock(block, 1, forwardDuration, 950, leftDuration, rightDuration, 0.4);
//
//            // When we move next block we need to move left as much as we have come right
//            leftDuration = rightDuration;
//        }
//    }

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

        //move left
        goLeft(wheelPower, leftDuration, "Going left");
        stopWithSleep("Pausing", 300);

        calibrateDirection(calibrationPower, 200);

        //servo up, release the block
        frontServo.setPosition(0.4);
        stop("Pausing");

        //move right - towards the depot
        goRight(1.0, rightDuration, "Going right");
        stopWithSleep("Pausing", 300);

        calibrateDirection(calibrationPower2, 300);

    }

    private void calibrateDirection(final double rightWheelPower, final int duration) {
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(rightWheelPower);
        rightback.setPower(rightWheelPower);
        sleep(duration);
    }


}