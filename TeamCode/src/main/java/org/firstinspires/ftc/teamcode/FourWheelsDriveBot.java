// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//@Autonomous(name="Drive Encoder2", group="Exercises")

public class FourWheelsDriveBot
{
    // Gobilda 435 rpm DC motor : Encoder Countable Events Per Revolution (Output Shaft) : 383.6 * 2 (2:1 belev gear ratio)
    static final double DRIVING_MOTOR_TICK_COUNT = 767;
    static final int DIRECTION_FORWARD = 1;
    static final int DIRECTION_BACKWARD = 2;
    static final int DIRECTION_LEFT = 3;
    static final int DIRECTION_RIGHT = 4;

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;
//    public DcMotor heavyDutyArm = null;

    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();
    private LinearOpMode opMode;

    public FourWheelsDriveBot(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void print(String message){
        String caption = "4WD";
        this.opMode.telemetry.addData(caption, message);
        this.opMode.telemetry.update();
    }

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftRear = hwMap.get(DcMotor.class, "leftRear");
        rightRear = hwMap.get(DcMotor.class, "rightRear");
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

//        heavyDutyArm = hwMap.get(DcMotor.class, "arm");

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

//        heavyDutyArm.setPower(0);
        print("Resetting Encoders");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        print(String.format("Starting at leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition()));

//        heavyDutyArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void testOneMotor(DcMotor motor, double speed, int direction){
        // reset the timeout time and start motion.
        runtime.reset();

        double timeoutS = 5.0;
        // make 3 turn
        int target = motor.getCurrentPosition() + (int)DRIVING_MOTOR_TICK_COUNT * 3 * direction;
        print(String.format("Start %s @ %7d", motor.getDeviceName(), motor.getCurrentPosition()));

        motor.setTargetPosition(target);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);

        while (this.opMode.opModeIsActive() && (runtime.seconds() < timeoutS) && motor.isBusy()) {
            // Display it for the driver.
            print(String.format("Running %s to %7d: @ %7d", motor.getDeviceName(), target, motor.getCurrentPosition()));
        }
        // Stop all motion;
        motor.setPower(0);

        print(String.format("Completed! %s @ %7d", motor.getDeviceName(), motor.getCurrentPosition()));

        this.opMode.sleep(3000);

    }

    public void driveStraightByDistance(int direction, float distance){
        // distance (in mm) = revolution * pi * diameter (100 mm)
        int target = (int)(distance / 3.1415 / 100 * DRIVING_MOTOR_TICK_COUNT);
        double power = 0.3;
        switch (direction){
            case DIRECTION_FORWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_BACKWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_LEFT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_RIGHT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
        }
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        while (this.opMode.opModeIsActive() && leftFront.isBusy()) {
            // Display it for the driver.
            print(String.format("Target : %7d @ leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                    target,
                    leftFront.getCurrentPosition(),
                    rightFront.getCurrentPosition(),
                    leftRear.getCurrentPosition(),
                    rightRear.getCurrentPosition()));
        }
        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        print(String.format("Arrive target : %7d @ leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                target,
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition()));
    }
}
