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

        moveBlocksOld();

        // TODO: New refactored code needs testing. If working start using this code and remove moveBlocksOld()
        // moveBlocksNew();
    }

    private void moveBlocksOld() {
        // Move Forward towards the blocks
        goForward(0.5, 1650, "Going forward");
        stop("Pausing");

        //Servo goes down
        frontServo.setPosition(0.0);

        //robot moves backward
        goBackward(0.5, 1050, "Going backward");
        stop("Pausing");

        //move left
        goLeft(0.5,3100, "Going left");
        stopWithSleep("Pausing", 300);

        //servo up
        frontServo.setPosition(0.4);
        stop("Pausing");

        //move right - towards the depot
        goRight(0.5,3700,"Going right");
        stopWithSleep("Pausing", 300);

        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0.4);
        rightback.setPower(0.4);
        sleep(300);

        // 2. Stone #2

        // Move Forward towards the blocks

        goForward(0.4, 1550, "Going forward");
        stop("Pausing");

        //Servo goes down
        frontServo.setPosition(0.0);


        //robot moves backward
        goBackward(0.5, 1050, "Going backward");
        stop("Pausing");

        //move left
        goLeft(0.5,3700, "Going right");
        stopWithSleep("Pausing", 300);

        //servo up
        frontServo.setPosition(0.4);

        //move right - towards the depot
        goRight(0.5,4300,"Going right");
        stopWithSleep("Pausing", 300);

        // 3. Stone #3

        // Move Forward towards the blocks
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0.4);
        rightback.setPower(0.4);
        sleep(300);

        goForward(0.4, 1550, "Going forward");
        stop("Pausing");

        //Servo goes down
        frontServo.setPosition(0.0);


        //robot moves backward
        goBackward(0.5, 1050, "Going backward");
        stop("Pausing");

        //move left
        goLeft(0.5,4300, "Going right");

        //servo up
        frontServo.setPosition(0.4);

        //move right - towards the depot
        goRight(0.5,4900,"Going right");
    }

    private void moveBlocksNew() {
        int leftDuration = 3100; // How much to move left to carry the block
        final int blockIncrement = 600; // When we return back we have to move right extra to pick next block

        for (int block = 1; block <= 6 ; block++) {
            // When we return back we have to move right extra to pick next block
            int rightDuration = leftDuration + blockIncrement;
            int forwardDuration = block == 1 ? 1650: 1550; // Move extra for 1st block
            moveBlock(0.5, forwardDuration, 1050, leftDuration, rightDuration);

            // When we move next block we need to move left as much as we have come right
            leftDuration = rightDuration;
        }
    }

    private void moveBlock(final double wheelPower,
                           final int forwardDuration,
                           final int backwardDuration,
                           final int leftDuration,
                           final int rightDuration) {
        // Move Forward towards the blocks
        goForward(wheelPower - 0.1, forwardDuration, "Going forward");
        stop("Pausing");

        //Servo goes down, to grab the block
        frontServo.setPosition(0.0);

        //robot moves backward
        goBackward(wheelPower, backwardDuration, "Going backward");
        stop("Pausing");

        // calibrateDirection(0.4, 300);

        //move left
        goLeft(wheelPower, leftDuration, "Going left");
        stopWithSleep("Pausing", 300);

        //servo up, release the block
        frontServo.setPosition(0.4);
        stop("Pausing");

        //move right - towards the depot
        goRight(wheelPower, rightDuration,"Going right");
        stopWithSleep("Pausing", 300);

        calibrateDirection(0.4, 300);
    }

    private void calibrateDirection(final double rightWheelPower, final int duration) {
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(rightWheelPower);
        rightback.setPower(rightWheelPower);
        sleep(duration);
    }
}
