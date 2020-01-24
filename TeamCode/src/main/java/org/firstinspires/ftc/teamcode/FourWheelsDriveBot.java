// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FourWheelsDriveBot
{

    // Gobilda 435 rpm DC motor : Encoder Countable Events Per Revolution (Output Shaft) : 383.6 * 2 (2:1 bevel gear ratio)
    static final double DRIVING_MOTOR_TICK_COUNT = 767;
    static final int DIRECTION_FORWARD = 1;
    static final int DIRECTION_BACKWARD = 2;
    static final int DIRECTION_LEFT = 3;
    static final int DIRECTION_RIGHT = 4;
    static final int DIRECTION_RQUARTER = 5;
    static final int DIRECTION_LQUARTER = 6;

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;





    HardwareMap hwMap = null;
    private ElapsedTime runtime = new ElapsedTime();
    private Orientation angles;
    private boolean arcadeMode = false;
    private double headingOffset = 0.0;
    protected LinearOpMode opMode;


    public FourWheelsDriveBot(LinearOpMode opMode) {
        this.opMode = opMode;
    }
// manual drive
    private double getRawHeading() {
        return angles.firstAngle;
    }

    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }


//    public void driveByHand(double _lf, double _lr, double _rf, double _rr) {
    public void driveByHand(double left_stick_x, double left_stick_y, double right_stick_x) {

        double drive  = -left_stick_y;
        double strafe = left_stick_x;
        double twist  = right_stick_x;


        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };

        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }


        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        leftFront.setPower(speeds[0]);
        rightFront.setPower(speeds[1]);
        leftRear.setPower(speeds[2]);
        rightRear.setPower(speeds[3]);



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

        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);

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

    }

    public void driveByVector(double vectorX, double vectorY){
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // the vector x positive value means right direction
        // the vector y positive value means backward direction
        //
        // assume rightFront and leftRear runs at same speed as a
        // assume leftFront and rightRear runs at same speed as b
        // 4x = -2a + 2b
        // -4y = 2a + 2b
        // =====>
        // a = -x - y
        // b = x - y
        double a = -vectorX - vectorY;
        double b = vectorX - vectorY;
        rightFront.setPower(a);
        leftFront.setPower(b);
        rightRear.setPower(b);
        leftRear.setPower(a);
        print(String.format("driveByVector(%.2f, %.2f) => leftFront|rightRear : %.2f, rightFront|leftRear : %.2f", vectorX, vectorY, b, a));
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
    public void driveStraightByDistance(int direction, double distance){
        // default max power 0.5
        driveStraightByDistance(direction, distance, 0.5);
    }

    public void driveStraightByDistance(int direction, double distance, double maxPower){
        // distance (in mm) = revolution * pi * diameter (100 mm)
        int target = (int)(distance / 3.1415 / 100 * DRIVING_MOTOR_TICK_COUNT);
        int startingPosition = leftFront.getCurrentPosition();
        switch (direction){
            case DIRECTION_FORWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_BACKWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_LEFT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition()- target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_RIGHT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_RQUARTER:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_LQUARTER:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            default:
                String msg = String.format("Unaccepted direction value (%d) for driveStraightByDistance()", direction);
                print(msg);
        }

        double power = maxPower;

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

    public void driveByDistanceWithAcceleration(int direction, double distance, double maxPower, int accelerationSteps){
        // distance (in mm) = revolution * pi * diameter (100 mm)
        int target = (int)(distance / 3.1415 / 100 * DRIVING_MOTOR_TICK_COUNT);
        int startingPosition = leftFront.getCurrentPosition();
        double accelerationDelta = maxPower/accelerationSteps;
        int accelerationInterval = 100;
        int realTarget;
        switch (direction){
            case DIRECTION_FORWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                realTarget = leftFront.getCurrentPosition() - target;
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            case DIRECTION_BACKWARD:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                realTarget = leftFront.getCurrentPosition() + target;
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_LEFT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() + target);
                realTarget = leftFront.getCurrentPosition() + target;
                rightFront.setTargetPosition(rightFront.getCurrentPosition() - target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() - target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() + target);
                break;
            case DIRECTION_RIGHT:
                leftFront.setTargetPosition(leftFront.getCurrentPosition() - target);
                realTarget = leftFront.getCurrentPosition() - target;
                rightFront.setTargetPosition(rightFront.getCurrentPosition() + target);
                leftRear.setTargetPosition(leftRear.getCurrentPosition() + target);
                rightRear.setTargetPosition(rightRear.getCurrentPosition() - target);
                break;
            default:
                realTarget = leftFront.getCurrentPosition() - target;
                String msg = String.format("Unaccepted direction value (%d) for driveStraightByDistance()", direction);
                print(msg);
        }

        double power = maxPower;

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setPower(accelerationDelta);
        rightFront.setPower(accelerationDelta);
        leftRear.setPower(accelerationDelta);
        rightRear.setPower(accelerationDelta);
        int step = 1;
        RobotLog.d("Let's go : Target: %d AccelerationDelta: %.2f CurrentPosition: %d Step: %d", realTarget, accelerationDelta, leftFront.getCurrentPosition(), step);
        while (this.opMode.opModeIsActive() && leftFront.isBusy()) {
            double distToDecelerate = Math.min(Math.abs(leftFront.getCurrentPosition() - startingPosition), accelerationSteps * accelerationInterval);
            RobotLog.d("In loop : CurrentPosition: %d Step: %d DistToDecelerate: %.2f", leftFront.getCurrentPosition(), step, distToDecelerate);

            if (Math.abs(leftFront.getCurrentPosition() - realTarget) > distToDecelerate &&
                    step < accelerationSteps &&
                    Math.abs(leftFront.getCurrentPosition() - startingPosition) > step * accelerationInterval) {

                RobotLog.d("Step up CurrentPosition: %d Step: %d DistToDecelerate: %.2f", leftFront.getCurrentPosition(), step, distToDecelerate);
                step++;
                leftFront.setPower(Math.max(accelerationDelta * step, maxPower));
                rightFront.setPower(Math.max(accelerationDelta * step, maxPower));
                leftRear.setPower(Math.max(accelerationDelta * step, maxPower));
                rightRear.setPower(Math.max(accelerationDelta * step, maxPower));


            }

            if (Math.abs(leftFront.getCurrentPosition() - realTarget) < distToDecelerate &&
                step > 1 && Math.abs(leftFront.getCurrentPosition() - realTarget) < step * accelerationInterval) {

                RobotLog.d("Step down CurrentPosition: %d Step: %d DistToDecelerate: %.2f", leftFront.getCurrentPosition(), step, distToDecelerate);
                step = (int)Math.floor(Math.abs(leftFront.getCurrentPosition() - realTarget) / accelerationInterval);
                leftFront.setPower(Math.max(accelerationDelta * step, 0.1));
                rightFront.setPower(Math.max(accelerationDelta * step, 0.1));
                leftRear.setPower(Math.max(accelerationDelta * step, 0.1));
                rightRear.setPower(Math.max(accelerationDelta * step, 0.1));

            }
        }
        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        print(String.format("Arrive target : %7d @ leftFront: %7d, rightFront:%7d, leftRear:%7d, rightRear:%7d",
                realTarget,
                leftFront.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                leftRear.getCurrentPosition(),
                rightRear.getCurrentPosition()));
    }
}

